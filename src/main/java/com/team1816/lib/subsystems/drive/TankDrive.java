package com.team1816.lib.subsystems.drive;

import static com.team1816.lib.util.driveUtil.DriveConversions.*;

import com.ctre.phoenix.motorcontrol.*;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.util.EnhancedMotorChecker;
import com.team1816.lib.util.team254.CheesyDriveHelper;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.lib.util.team254.SwerveDriveSignal;
import com.team1816.season.configuration.Constants;
import com.team1816.season.hardware.components.ProxySensor;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.LedManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.ArrayList;

@Singleton
public class TankDrive extends Drive implements DifferentialDrivetrain {

    /** Components */
    private final IGreenMotor leftMain, rightMain;
    private final IGreenMotor leftFollowerA, rightFollowerA, leftFollowerB, rightFollowerB;

    /** Odometry */
    private DifferentialDriveOdometry tankOdometry;
    private static final DifferentialDriveKinematics tankKinematics = new DifferentialDriveKinematics(
        kDriveWheelTrackWidthMeters
    );
    private final CheesyDriveHelper driveHelper = new CheesyDriveHelper();
    private final double tickRatioPerLoop = Constants.kLooperDt / .1d; // Convert Ticks/100MS into Ticks/Robot Loop

    /** States */
    public double leftPowerDemand, rightPowerDemand; // % Output (-1 to 1) - used in OPEN_LOOP
    public double leftVelDemand, rightVelDemand; // Velocity (Ticks/100MS) - used in TRAJECTORY_FOLLOWING

    private double leftActualDistance = 0, rightActualDistance = 0; // Meters

    private double leftActualVelocity, rightActualVelocity; // Ticks/100MS

    double leftErrorClosedLoop;
    double rightErrorClosedLoop;

    /**
     * Instantiates a swerve drivetrain from base subsystem parameters
     * @param lm LEDManager
     * @param inf Infrastructure
     * @param rs RobotState
     * @see Drive#Drive(LedManager, Infrastructure, RobotState)
     */
    @Inject
    public TankDrive(LedManager lm, Infrastructure inf, RobotState rs) {
        super(lm, inf, rs);
        // configure motors
        leftMain = factory.getMotor(NAME, "leftMain");
        leftFollowerA = factory.getFollowerMotor(NAME, "leftFollower", leftMain);
        leftFollowerB = factory.getFollowerMotor(NAME, "leftFollowerTwo", leftMain);
        rightMain = factory.getMotor(NAME, "rightMain");
        rightFollowerA = factory.getFollowerMotor(NAME, "rightFollower", rightMain);
        rightFollowerB = factory.getFollowerMotor(NAME, "rightFollowerTwo", rightMain);

        // configure follower motor currentLimits
        var currentLimitConfig = new SupplyCurrentLimitConfiguration(
            true,
            factory.getConstant(NAME, "currentLimit", 40),
            0,
            0
        );
        leftMain.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        leftFollowerA.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        leftFollowerB.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        rightMain.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        rightFollowerA.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );
        rightFollowerB.configSupplyCurrentLimit(
            currentLimitConfig,
            Constants.kLongCANTimeoutMs
        );

        setOpenLoop(DriveSignal.NEUTRAL);

        tankOdometry =
            new DifferentialDriveOdometry(
                getActualHeading(),
                leftActualDistance,
                rightActualDistance
            );
    }

    /**
     * Read/Write Periodic
     */

    /**
     * Writes outputs / demands to hardware on the drivetrain such as motors and handles the desired state of the left
     * and right sides. Directly writes to the motors.
     * @see IGreenMotor
     */
    @Override
    public synchronized void writeToHardware() { // sets the demands for hardware from the inputs provided
        if (controlState == ControlState.OPEN_LOOP) {
            leftMain.set(
                ControlMode.PercentOutput,
                isSlowMode ? (leftPowerDemand * 0.5) : leftPowerDemand
            );
            rightMain.set(
                ControlMode.PercentOutput,
                isSlowMode ? (rightPowerDemand * 0.5) : rightPowerDemand
            );
        } else {
            leftMain.set(ControlMode.Velocity, leftVelDemand);
            rightMain.set(ControlMode.Velocity, rightVelDemand);
        }
    }

    /**
     * Reads outputs from hardware on the drivetrain such as sensors and handles the actual state the wheels and
     * drivetrain speeds. Used to update odometry and other related data.
     * @see Infrastructure
     * @see RobotState
     */
    @Override
    public synchronized void readFromHardware() {
        // update current motor velocities and distance traveled
        leftActualVelocity = leftMain.getSelectedSensorVelocity(0);
        rightActualVelocity = rightMain.getSelectedSensorVelocity(0);
        leftActualDistance += ticksToMeters(leftActualVelocity * tickRatioPerLoop);
        rightActualDistance += ticksToMeters(rightActualVelocity * tickRatioPerLoop);

        // update error (only if in closed loop where knowing it is useful)
        if (controlState == ControlState.TRAJECTORY_FOLLOWING) {
            leftErrorClosedLoop = leftMain.getClosedLoopError(0);
            rightErrorClosedLoop = rightMain.getClosedLoopError(0);
        }

        // update current movement of the whole drivetrain
        chassisSpeed =
            tankKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(getLeftMPSActual(), getRightMPSActual())
            );

        // update actual heading from gyro (pigeon)
        if (RobotBase.isSimulation()) {
            simulateGyroOffset();
        }
        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());

        robotState.relativeDriveTrainAltitude = infrastructure.getMaxDistance();

        tankOdometry.update(actualHeading, leftActualDistance, rightActualDistance);
        updateRobotState();
    }

    /** Config */

    /**
     * Zeroes the encoders and odometry based on a certain pose
     * @param pose Pose2d
     * @see Drive#zeroSensors()
     */
    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");

        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());
        resetEncoders();
        resetOdometry(pose);
        startingPose = pose;

        chassisSpeed = new ChassisSpeeds();
        isBraking = false;
    }

    /**
     * Stops the drivetrain
     * @see Drive#stop()
     */
    @Override
    public synchronized void stop() {
        setOpenLoop(
            controlState == ControlState.OPEN_LOOP
                ? DriveSignal.NEUTRAL
                : DriveSignal.BRAKE
        );
        setBraking(controlState == ControlState.TRAJECTORY_FOLLOWING);
    }

    /**
     * Resets the encoders to hold the zero value
     * @see this#zeroSensors(Pose2d)
     */
    public synchronized void resetEncoders() {
        leftMain.setSelectedSensorPosition(0, 0, 0);
        rightMain.setSelectedSensorPosition(0, 0, 0);
        leftActualDistance = 0;
        rightActualDistance = 0;
    }

    /**
     * Resets the odometry to a certain pose
     * @param pose Pose2d
     */
    @Override
    public void resetOdometry(Pose2d pose) {
        tankOdometry.resetPosition(
            getActualHeading(),
            leftActualDistance,
            rightActualDistance,
            pose
        );
        tankOdometry.update(actualHeading, leftActualDistance, rightActualDistance);
        updateRobotState();
    }

    /**
     * Updates robotState based on values from odometry and sensor readings in readFromHardware
     * @see RobotState
     */
    @Override
    public void updateRobotState() {
        robotState.fieldToVehicle = tankOdometry.getPoseMeters();

        var cs = new ChassisSpeeds(
            chassisSpeed.vxMetersPerSecond,
            chassisSpeed.vyMetersPerSecond,
            chassisSpeed.omegaRadiansPerSecond
        );
        robotState.calculatedVehicleAccel =
            new ChassisSpeeds(
                (cs.vxMetersPerSecond - robotState.deltaVehicle.vxMetersPerSecond) /
                Constants.kLooperDt,
                (cs.vyMetersPerSecond - robotState.deltaVehicle.vyMetersPerSecond) /
                Constants.kLooperDt,
                cs.omegaRadiansPerSecond - robotState.deltaVehicle.omegaRadiansPerSecond
            );
        robotState.deltaVehicle = cs;
    }

    /** Open loop control */

    /**
     * Sets open loop percent output commands based on the DriveSignal from setTeleOpInputs()
     * @param signal DriveSignal
     * @see Drive#setOpenLoop(DriveSignal)
     */
    @Override
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (controlState != ControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            controlState = ControlState.OPEN_LOOP;
            leftErrorClosedLoop = 0;
            rightErrorClosedLoop = 0;
        }
        leftPowerDemand = signal.getLeft();
        rightPowerDemand = signal.getRight();

        leftVelDemand = leftPowerDemand * maxVelTicks100ms;
        rightVelDemand = rightPowerDemand * maxVelTicks100ms;
    }

    /**
     * Translates teleoperated inputs into a DriveSignal to be used in setTeleOpInputs()
     * @param forward forward demand
     * @param strafe strafe demand
     * @param rotation rotation demand
     * @see this#setOpenLoop(DriveSignal)
     * @see Drive#setTeleopInputs(double, double, double)
     * @see SwerveDriveSignal
     */
    @Override
    public void setTeleopInputs(double forward, double strafe, double rotation) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        DriveSignal driveSignal = driveHelper.cheesyDrive(
            (isDemoMode ? forward * demoModeMultiplier : forward),
            (isDemoMode ? rotation * demoModeMultiplier : rotation),
            false,
            false
        );

        // To avoid overriding brake command
        if (!isBraking) {
            setOpenLoop(driveSignal);
        }
    }

    /**
     * Adapts a DriveSignal for closed loop PID controlled motion
     * @param signal DriveSignal
     */
    public synchronized void setVelocity(DriveSignal signal) {
        if (controlState == ControlState.OPEN_LOOP) {
            System.out.println("Switching to Velocity");

            leftMain.selectProfileSlot(0, 0);
            rightMain.selectProfileSlot(0, 0);

            leftMain.configNeutralDeadband(0.0, 0);
            rightMain.configNeutralDeadband(0.0, 0);
        }

        leftVelDemand = signal.getLeft();
        rightVelDemand = signal.getRight();
    }

    /**
     * Utilizes a DriveSignal to adapt Trajectory demands for TRAJECTORY_FOLLOWING and closed loop control
     * @param leftVel left velocity
     * @param rightVel right velocity
     */
    public void updateTrajectoryVelocities(Double leftVel, Double rightVel) {
        // Velocities are in m/sec comes from trajectory command
        var signal = new DriveSignal(
            metersPerSecondToTicksPer100ms(leftVel),
            metersPerSecondToTicksPer100ms(rightVel)
        );
        setVelocity(signal);
    }

    /**
     * General getters and setters
     */

    /**
     * Sets whether the drivetrain is braking
     * @param braking boolean
     * @see Drive#setBraking(boolean)
     */
    @Override
    public synchronized void setBraking(boolean braking) {
        if (isBraking != braking) {
            System.out.println("braking: " + braking);
            isBraking = braking;

            if (braking) {
                leftMain.set(ControlMode.Velocity, 0);
                rightMain.set(ControlMode.Velocity, 0);
            }
            // TODO ensure that changing neutral modes won't backfire while we're using brushless motors
            NeutralMode mode = braking ? NeutralMode.Brake : NeutralMode.Coast;

            rightMain.setNeutralMode(mode);
            rightFollowerA.setNeutralMode(mode);
            rightFollowerB.setNeutralMode(mode);

            leftMain.setNeutralMode(mode);
            leftFollowerA.setNeutralMode(mode);
            leftFollowerB.setNeutralMode(mode);
        }
    }

    /**
     * Returns the actual velocity of the left side in meters per second
     * @return left velocity (m/s)
     * @see com.team1816.lib.util.driveUtil.DriveConversions#ticksPer100MSToMPS(double)
     */
    public double getLeftMPSActual() {
        return ticksPer100MSToMPS(leftActualVelocity);
    }

    /**
     * Returns the actual velocity of the right side in meters per second
     * @return right velocity (m/s)
     * @see com.team1816.lib.util.driveUtil.DriveConversions#ticksPer100MSToMPS(double)
     */
    public double getRightMPSActual() {
        return ticksPer100MSToMPS(rightActualVelocity);
    }

    /**
     * Returns the left velocity demand in ticks per 100ms
     * @return leftVelDemand
     */
    @Override
    public double getLeftVelocityTicksDemand() {
        return leftVelDemand;
    }

    /**
     * Returns the right velocity demand in ticks per 100ms
     * @return rightVelDemand
     */
    @Override
    public double getRightVelocityTicksDemand() {
        return rightVelDemand;
    }

    /**
     * Returns the actual left velocity in ticks per 100ms
     * @return leftVelActual
     * @see IMotorController
     * @see IGreenMotor
     */
    @Override
    public double getLeftVelocityTicksActual() {
        return leftMain.getSelectedSensorVelocity(0);
    }

    /**
     * Returns the actual right velocity in ticks per 100ms
     * @return rightVelActual
     * @see IMotorController
     * @see IGreenMotor
     */
    @Override
    public double getRightVelocityTicksActual() {
        return rightMain.getSelectedSensorVelocity(0);
    }

    /**
     * Returns the total distance (not displacement) traveled by the left side of the drivetrain
     * @return leftActualDistance
     */
    @Override
    public double getLeftDistance() {
        return leftActualDistance;
    }

    /**
     * Returns the total distance (not displacement) traveled by the right side of the drivetrain
     * @return rightActualDistance
     */
    @Override
    public double getRightDistance() {
        return rightActualDistance;
    }

    /**
     * Returns the left side closed loop error (in-built)
     * @return leftErrorClosedLoop
     */
    @Override
    public double getLeftError() {
        return leftErrorClosedLoop;
    }

    /**
     * Returns the right side closed loop error (in-built)
     * @return rightErrorClosedLoop
     */
    @Override
    public double getRightError() {
        return rightErrorClosedLoop;
    }

    /** config and tests */

    /**
     * Tests the drivetrain by seeing if each side can go back and forth
     * @return true if tests passed
     * @see Drive#testSubsystem()
     */
    @Override
    public boolean testSubsystem() {
        boolean leftSide = EnhancedMotorChecker.checkMotor(this, leftMain);
        boolean rightSide = EnhancedMotorChecker.checkMotor(this, rightMain);

        boolean checkPigeon = infrastructure.getPigeon() == null;

        System.out.println(leftSide && rightSide && checkPigeon);
        if (leftSide && rightSide && checkPigeon) {
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
        } else {
            ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
        }
        return leftSide && rightSide;
    }

    /**
     * Returns the pid configuration of the motors
     * @return PIDSlotConfiguration
     * @see Drive#getPIDConfig()
     */
    @Override
    public PIDSlotConfiguration getPIDConfig() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kP = 0.0;
        defaultPIDConfig.kI = 0.0;
        defaultPIDConfig.kD = 0.0;
        defaultPIDConfig.kF = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory.getSubsystem(NAME).pidConfig.getOrDefault("slot0", defaultPIDConfig)
            : defaultPIDConfig;
    }

    /**
     * Returns the associated kinematics with the drivetrain
     * @return tankKinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return tankKinematics;
    }
}
