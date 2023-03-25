package com.team1816.lib.subsystems.turret;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.IMotorSensor;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class models a multi-functional turret with target and field tracking abilities
 */
@Singleton
public class Turret extends Subsystem implements PidProvider {

    /**
     * Properties
     */
    public static final String NAME = "turret";
    public static final double kJogSpeed = 0.5;
    public static final double kSouth = 0; // deg - relative to vehicle NOT FIELD
    public static final double kEast = 270; // deg - relative to vehicle NOT FIELD
    public static final double kNorth = 180; // deg - relative to vehicle NOT FIELD
    public static final double kWest = 90; // deg - relative to vehicle NOT FIELD
    public final int kRevLimit;
    public final int kFwdLimit;
    public static int kRevWrapAroundPos; // lowest allowed tick value before turret masks (+turretPPR)
    public static int kFwdWrapAroundPos; // highest allowed tick value before turret masks (-turretPPR)
    public final int kAbsTicksSouthOffset; // abs encoder count at cardinal SOUTH
    public static int kAbsPPR;
    public static int kTurretPPR;
    private final double kRatioTurretAbs;
    public final double kDeltaXScalar;

    private static final int kPrimaryCloseLoop = 0;
    private final PIDSlotConfiguration pidConfig;

    /**
     * Components
     */
    private final IGreenMotor turretMotor;

    private static LedManager led;

    /**
     * State
     */
    private int desiredPos = 0;
    private int followingPos = 0;
    private boolean lostEncPos = false;
    private boolean deadzone = false;
    private int visionCorroboration = 0;
    private double turretSpeed; // this is used as a percent output demand, NOT THE SAME AS turretVelocity
    private double turretVelocity = 0d; // used to calculate acceleration and other gyroscopic effects
    private double turretRotationalAcceleration = 0d;
    private double turretCentripetalAcceleration = 0d;
    private boolean outputsChanged = true;
    private Pose2d target = Constants.targetPos;
    private ControlMode controlMode;

    /**
     * Instantiates a turret with a camera (only used for feedback loop based automatic homing) and standard subsystem components
     *
     * @param ledManager LEDManager
     * @param inf        Infrastructure
     * @param rs         RobotState
     */
    @Inject
    public Turret(
        LedManager ledManager,
        Infrastructure inf,
        RobotState rs
    ) {
        super(NAME, inf, rs);
        led = ledManager;
        turretMotor = factory.getMotor(NAME, "turretMotor");
        kDeltaXScalar = factory.getConstant(NAME, "deltaXScalar", 1);

        // Define PPR values and determine whether to offset set positions by absEnc south pos
        kAbsPPR = (int) factory.getConstant(NAME, "absPPR");
        kTurretPPR = (int) factory.getConstant(NAME, "turretPPR");
        kRatioTurretAbs = (double) kTurretPPR / kAbsPPR;
        kAbsTicksSouthOffset =
            kRatioTurretAbs == 1
                ? ((int) factory.getConstant(NAME, "absPosTicksSouth"))
                : 0;

        // define limits + when turret should wrap around
        kRevLimit =
            Math.min(
                (int) factory.getConstant(NAME, "fwdLimit"),
                (int) factory.getConstant(NAME, "revLimit")
            );
        kFwdLimit =
            Math.max(
                (int) factory.getConstant(NAME, "fwdLimit"),
                (int) factory.getConstant(NAME, "revLimit")
            );
        int MASK = Math.abs((kRevLimit + kTurretPPR) - (kFwdLimit)) / 2; // this value is truncated
        kFwdWrapAroundPos = kFwdLimit + MASK;
        kRevWrapAroundPos = kRevLimit - MASK;

        // Position Control
        double peakOutput = 0.75;
        pidConfig = factory.getPidSlotConfig(NAME);
        turretMotor.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        turretMotor.configNominalOutputForward(0, Constants.kCANTimeoutMs);
        turretMotor.configNominalOutputReverse(0, Constants.kCANTimeoutMs);
        turretMotor.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);

        // Soft Limits
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turretMotor.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        turretMotor.configForwardSoftLimitThreshold(kFwdLimit, Constants.kCANTimeoutMs); // Forward = MAX
        turretMotor.configReverseSoftLimitThreshold(kRevLimit, Constants.kCANTimeoutMs); // Reverse = MIN
        turretMotor.overrideLimitSwitchesEnable(true);
        turretMotor.overrideSoftLimitsEnable(true);

        if (Constants.kLoggingRobot) {
            desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Turret/DesiredPosition");
            actStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Turret/ActualPosition");
        }
    }

    /**
     * converts 0-360 to 0-TURRET_ENCODER_PPR
     */
    public static int convertTurretDegreesToTicks(double degrees) {
        return (int) (degrees / 360.0 * kTurretPPR);
    }

    /**
     * converts 0-TURRET_ENCODER_PPR
     */
    public static double convertTurretTicksToDegrees(double ticks) {
        return ticks / kTurretPPR * 360;
    }

    /**
     * Zeroes the turret position
     */
    @Override
    public synchronized void zeroSensors() {
        zeroSensors(false);
    }

    /**
     * Zeroes the turret depending on if the encoder needs to be reset and handles for gear ratios
     *
     * @param resetEncPos boolean
     */
    public synchronized void zeroSensors(boolean resetEncPos) {
        desiredPos = 0;
        followingPos = 0;
        lostEncPos = false;

        if ((int) kRatioTurretAbs == 1) {
            var sensor = ((IMotorSensor) turretMotor);
            var sensorVal = sensor.getPulseWidthPosition() % kAbsPPR;
            sensor.setQuadraturePosition(sensorVal);
            System.out.println("zeroing turret at " + sensorVal);
        } else {
            if (resetEncPos) {
                turretMotor.setSelectedSensorPosition(
                    0,
                    kPrimaryCloseLoop,
                    Constants.kCANTimeoutMs
                );
            }
        }
    }

    /**
     * Returns the ControlMode of the turret
     *
     * @return controlMode
     */
    public ControlMode getControlMode() {
        return controlMode;
    }

    /**
     * Sets the ControlMode of the turret
     *
     * @param controlMode ControlMode
     */
    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode != controlMode) {
            outputsChanged = true;
            this.controlMode = controlMode;
            System.out.println("turret controlMode: " + this.controlMode);
        }
    }

    /**
     * Sets the turret speed to a value (only used for continually constrained motion profiling)
     *
     * @param velocity turretVelocity
     */
    public void setTurretVelocity(double velocity) {
        setControlMode(ControlMode.MANUAL);
        if (turretSpeed != velocity) {
            turretSpeed = velocity;
            outputsChanged = true;
        }
    }

    /**
     * Sets the desired position of the turret
     *
     * @param position desiredPosition
     */
    private synchronized void setDesiredPos(double position) {
        if (desiredPos != (int) position) {
            desiredPos = (int) position;
            outputsChanged = true;
        }
    }

    // CCW positive - 0 to 360

    /**
     * Sets the turret to a position based on an angle
     *
     * @param angle (degrees)
     * @see this#setDesiredPos(double)
     */
    public synchronized void setTurretAngle(double angle) {
        setControlMode(ControlMode.POSITION);
        System.out.println("setting turret angle: " + angle);
        setDesiredPos(convertTurretDegreesToTicks(angle));
        followingPos = desiredPos;
    }

    /**
     * Sets the turret to follow a certain angle with gyroscopic correction
     *
     * @param angle (degrees)
     */
    public synchronized void setFollowingAngle(double angle) {
        setDesiredPos(convertTurretDegreesToTicks(angle));
    }

    /**
     * Sets the target for the turret to track based on a target pose on the field
     *
     * @param target Pose2d
     */
    public void setTarget(Pose2d target) {
        if (this.target != target) {
            this.target = target;
        }
    }

    /**
     * Snaps the turret to its target position by setting its controlMode to TARGET_FOLLOWING
     */
    public synchronized void snap() {
        setControlMode(ControlMode.TARGET_FOLLOWING);
    }

    /**
     * Locks the turret to a certain position to be held
     */
    public synchronized void lockTurret() {
        setTurretAngle(getActualPosDegrees());
    }

    /**
     * Returns the actual angle of the turret in degrees
     *
     * @return angle (degrees)
     */
    public double getActualPosDegrees() {
        return convertTurretTicksToDegrees(getActualPosTicks());
    }

    /**
     * Returns the actual position of the turret in encoder ticks based on the encoder reading
     *
     * @return actualTurretPos
     */
    public double getActualPosTicks() {
        return (
            (
                turretMotor.getSelectedSensorPosition(kPrimaryCloseLoop) -
                    kAbsTicksSouthOffset
            )
        );
    }

    /**
     * Returns the desired position of the turret in encoder ticks
     *
     * @return desiredTurretPos
     */
    public double getDesiredPosTicks() {
        if (controlMode == ControlMode.POSITION) {
            return desiredPos;
        }
        return followingPos;
    }

    /**
     * Returns the error in the actual and desired positions of the turret
     *
     * @return error (encoder ticks)
     */
    public double getPosError() {
        return getDesiredPosTicks() - getActualPosTicks();
    }

    /** periodic */

    /**
     * Reads position and velocity from the encoder and updates RobotState
     *
     * @see RobotState
     */
    @Override
    public void readFromHardware() {
        if (followingPos > 2 * kTurretPPR) {
            followingPos %= kTurretPPR;
        }

        deadzone =
            (followingPos >= kFwdWrapAroundPos || followingPos <= kRevWrapAroundPos);
        outputToSmartDashboard();

        double sensorVel = turretMotor.getSelectedSensorVelocity(0) / 10d;
        turretRotationalAcceleration =
            Units.degreesToRadians(
                convertTurretTicksToDegrees(sensorVel - turretVelocity) /
                    Constants.kLooperDt
            );
        turretCentripetalAcceleration =
            Math.pow(Units.degreesToRadians(convertTurretTicksToDegrees(sensorVel)), 2) *
                Constants.kTurretZedRadius;
        turretVelocity = sensorVel;

        if (turretMotor.hasResetOccurred()) {
            System.out.println("turretMotor lost its position!");
            led.setDefaultStatus(LedManager.RobotStatus.ERROR);
            lostEncPos = true;
        }

        robotState.vehicleToTurret = Rotation2d.fromDegrees(getActualPosDegrees());
        robotState.fieldToTurret =
            new Pose2d(
                robotState.fieldToVehicle
                    .transformBy(
                        new Transform2d(
                            Constants.kTurretMountingOffset,
                            Constants.EmptyRotation2d
                        )
                    )
                    .getTranslation(),
                robotState.fieldToVehicle.getRotation().plus(robotState.vehicleToTurret)
            );

        if (Constants.kLoggingRobot) {
            ((DoubleLogEntry) desStatesLogger).append(followingPos);
            ((DoubleLogEntry) actStatesLogger).append(getActualPosTicks());
        }
    }

    /**
     * Writes output to the {@link Turret#turretMotor} based on the controlMode
     *
     * @see Turret#controlMode
     */
    @Override
    public void writeToHardware() {
        switch (controlMode) {
            case FIELD_FOLLOWING:
                trackGyro();
                positionControl(followingPos);
                break;
            case TARGET_FOLLOWING:
                setTarget(Constants.fieldCenterPose);
                trackTarget();
                positionControl(followingPos);
                break;
            case ABSOLUTE_FOLLOWING:
                trackAbsolute();
                positionControl(followingPos);
                break;
            case EJECT:
                eject();
                positionControl(followingPos);
                break;
            case POSITION:
                positionControl(desiredPos);
                break;
            case MANUAL:
                manualControl();
                break;
        }
    }

    /**
     * A state based control switch for revolving between controlModes
     *
     * @see Turret#controlMode
     */
    public void revolve() {
        switch (controlMode) {
            case FIELD_FOLLOWING:
                setControlMode(ControlMode.EJECT);
                break;
            case EJECT:
                setControlMode(ControlMode.TARGET_FOLLOWING);
                break;
            case TARGET_FOLLOWING:
                setControlMode(ControlMode.ABSOLUTE_FOLLOWING);
                break;
            case ABSOLUTE_FOLLOWING:
                setControlMode(ControlMode.FIELD_FOLLOWING);
                break;
            case POSITION:
                setControlMode(ControlMode.FIELD_FOLLOWING);
                break;
            case MANUAL:
                setControlMode(ControlMode.FIELD_FOLLOWING);
                break;
        }
    }

    /** offsets */

    /**
     * Gyroscopic offset that preserves the angle of the turret with respect to the field irrespective of the rotation of the drivetrain
     *
     * @return int offset
     */
    private int fieldFollowingOffset() {
        return -convertTurretDegreesToTicks( // this is currently negated because motor is running counterclockwise
            robotState.fieldToVehicle.getRotation().getDegrees()
        );
    }

    /**
     * An offset that accounts for tracking a target based on its pose
     *
     * @return int offset
     */
    private int targetFollowingOffset() {
        double opposite = target.getY() - robotState.getFieldToTurretPos().getY();
        double adjacent = target.getX() - robotState.getFieldToTurretPos().getX();
        double turretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) turretAngle += Math.PI;
        return convertTurretDegreesToTicks(Units.radiansToDegrees(turretAngle));
    }

    /**
     * An offset that accounts for tracking a target and motion differences of the turret to provide the most accurate
     * and continuous tracking
     *
     * @return int offset
     */
    private int estimatedTargetFollowingOffset() {
        double opposite =
            target.getY() - robotState.getEstimatedFieldToTurretPos().getY();
        double adjacent =
            target.getX() - robotState.getEstimatedFieldToTurretPos().getX();
        double turretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) turretAngle += Math.PI;
        return convertTurretDegreesToTicks(Units.radiansToDegrees(turretAngle));
    }

    /** actions for modes */

    /**
     * Sets the desired position of the turret for constant gyroscopic tracking of a direction
     *
     * @see Turret#fieldFollowingOffset()
     */
    private void trackGyro() {
        int fieldTickOffset = fieldFollowingOffset();
        int adj = (desiredPos + fieldTickOffset);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    /**
     * Sets the desired position of the turret for the tracking of a target
     *
     * @see Turret#targetFollowingOffset()
     */
    private void trackTarget() {
        int fieldTickOffset = fieldFollowingOffset();
        int targetOffset = targetFollowingOffset();

        int adj = (desiredPos + fieldTickOffset + targetOffset + visionCorroboration);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    /**
     * Sets the desired position of the turret for the tracking of a target whilst accounting for motion
     *
     * @see Turret#estimatedTargetFollowingOffset()
     */
    private void trackAbsolute() {
        int fieldTickOffset = fieldFollowingOffset();
        int targetOffset = estimatedTargetFollowingOffset();

        int adj = (desiredPos + fieldTickOffset + targetOffset);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    /**
     * Sets the desired position of the turret to always aim away from the target with minimal effort
     *
     * @see Turret#estimatedTargetFollowingOffset()
     */
    private void eject() {
        int fieldTickOffset = fieldFollowingOffset();
        int targetOffset = estimatedTargetFollowingOffset();
        int throwOffset = convertTurretDegreesToTicks(30);

        int adj = (desiredPos + fieldTickOffset + targetOffset + throwOffset);
        if (adj != followingPos) {
            followingPos = adj;
            outputsChanged = true;
        }
    }

    /**
     * Sets the output of the turret based on the desired position
     *
     * @param pos position
     * @see Turret#writeToHardware()
     */
    private void positionControl(int pos) {
        if (outputsChanged) {
            outputsChanged = false;
            if (lostEncPos) {
                manualControl();
                return;
            }

            if (pos > kFwdWrapAroundPos) {
                pos -= kTurretPPR;
            } else if (pos < kRevWrapAroundPos) {
                pos += kTurretPPR;
            }
            int rawPos = (pos + kAbsTicksSouthOffset);

            turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, rawPos);
        }
    }

    /**
     * Manually controls the turret motor with percent output open loop control
     */
    private void manualControl() {
        if (outputsChanged) {
            turretMotor.set(
                com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput,
                turretSpeed
            );
            outputsChanged = false;
        }
    }

    /** Config and Misc */

    /**
     * Stops the turret
     */
    @Override
    public void stop() {
    }

    /**
     * Returns the pid configuration of the turret motor(s)
     *
     * @return pidConfig
     */
    @Override
    public PIDSlotConfiguration getPIDConfig() {
        return pidConfig;
    }

    /**
     * Tests the turret subsystem by moving the turret to its limits and checking the position
     *
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        boolean passed;
        turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, .2);
        Timer.delay(2);
        var ticks = getActualPosTicks();
        var diff = Math.abs(ticks - kFwdLimit);
        System.out.println(" + TICKS: " + ticks + "  ERROR: " + diff);
        passed = diff <= 50;
        turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, -.2);
        Timer.delay(2);
        ticks = getActualPosTicks();
        diff = Math.abs(ticks - kRevLimit);
        System.out.println(" - TICKS: " + ticks + "  ERROR: " + diff);
        passed = passed & diff <= 50;
        turretMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
        return passed;
    }

    /**
     * Outputs a dead-zone boolean to the SmartDahsboard if the turret is at its limits and there is trying to follow a
     * position that lies in the deadzone
     */
    public void outputToSmartDashboard() {
        SmartDashboard.putString("Turret/Deadzone", deadzone ? "Deadzone" : "Free");
    }

    /**
     * Control Modes
     */
    public enum ControlMode {
        FIELD_FOLLOWING,
        TARGET_FOLLOWING,
        ABSOLUTE_FOLLOWING,
        EJECT,
        POSITION,
        MANUAL,
    }
}
