package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.simUtil.SingleJointedElevatorArmSim;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class Elevator extends Subsystem {

    /**
     * Name
     */
    private static final String NAME = "elevator";

    /**
     * Components
     */
    private final IGreenMotor angleMotorMain;
    private final IGreenMotor angleMotorFollower;
    private final IGreenMotor extensionMotor;
    /**
     * Properties
     */

    // where ur drawing stuff
    private final Mechanism2d mechCanvas = new Mechanism2d(3, 3);
    private final MechanismRoot2d root = mechCanvas.getRoot("ElevatorArm", 1.25, 0.5);
    private final MechanismLigament2d simArm = root.append(new MechanismLigament2d("elevator", kElevatorMinLength, 90));
    private static final double kElevatorMinLength = 0.70; // meters
    private static final double kElevatorMaxLength = 1.25; // meters
    private static final double kArmGearing = 250; // meters
    public static final double kArmMass = 13.60; // kg

    public static final double angleTicksPerDegree = factory.getConstant(NAME, "angleTicksPerDegree", 0);
    public static final double stowPos = factory.getConstant(NAME, "stowAngle") * angleTicksPerDegree;
    public static final double collectPos = factory.getConstant(NAME, "collectAngle") * angleTicksPerDegree;
    public static final double scorePos = factory.getConstant(NAME, "scoreAngle") * angleTicksPerDegree;

    public static final double shelfPos = factory.getConstant(NAME, "shelfAngle") * angleTicksPerDegree;

    public static final double extensionTicksPerInch = factory.getConstant(NAME, "extensionTicksPerInch", 0);
    public static final double minExtension = factory.getConstant(NAME, "minExtension") * extensionTicksPerInch;
    public static final double midExtension = factory.getConstant(NAME, "midExtension") * extensionTicksPerInch;
    public static final double maxExtension = factory.getConstant(NAME, "maxExtension") * extensionTicksPerInch;
    public static final double shelfExtension = factory.getConstant(NAME, "shelfExtension") * extensionTicksPerInch;

    private final double allowableAngleError;
    private final double allowableExtensionError;

    private static double maxAngularVelocity; // rad/s
    private static double maxAngularAcceleration; // rad/s^2
    private static double maxExtendedAngularAcceleration; // rad/s^2
    private static double maxExtensionVelocity; // m/s
    private static double maxExtensionAcceleration; // m/s^2
    private final int movingArmSlot = 0;
    private final int lockedArmSlot = 1;
    private final int extensionPIDSlot = 2;
    private final boolean motionMagicEnabled;

    /**
     * States
     */

    private double desiredExtensionTicks = 0;
    private double desiredAngleTicks = 0;
    private double actualExtensionTicks = 0;
    private double actualAngleTicks = 0;
    private double actualAngleThetaDegrees;
    private double actualExtensionInches;
    private double actualAngleVel;
    private double actualExtensionVel;
    private ANGLE_STATE desiredAngleState = ANGLE_STATE.STOW;
    private EXTENSION_STATE desiredExtensionState = EXTENSION_STATE.MIN;

    private boolean angleOutputsChanged;
    private boolean extensionOutputsChanged;

    /**
     * Logging
     */
    private DoubleLogEntry desiredExtensionLogger;
    private DoubleLogEntry actualExtensionLogger;
    private DoubleLogEntry armCurrentDraw;
    private DoubleLogEntry extensionCurrentDraw;


    /**
     * Base constructor needed to instantiate a subsystem
     *
     * @param inf Infrastructure
     * @param rs  RobotState
     */
    @Inject
    public Elevator(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        // components
        this.angleMotorMain = factory.getMotor(NAME, "angleMotorMain");
        this.angleMotorFollower = factory.getFollowerMotor(NAME, "angleMotorFollower", angleMotorMain);
        this.extensionMotor = factory.getMotor(NAME, "extensionMotor");


        double extensionPeakOutput = 1;
        extensionMotor.configPeakOutputForward(extensionPeakOutput, Constants.kCANTimeoutMs);
        extensionMotor.configPeakOutputReverse(-extensionPeakOutput, Constants.kCANTimeoutMs);
        extensionMotor.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        extensionMotor.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        extensionMotor.configForwardSoftLimitThreshold(factory.getConstant(NAME, "forwardExtensionLimit") * extensionTicksPerInch, Constants.kCANTimeoutMs);
        extensionMotor.configReverseSoftLimitThreshold(factory.getConstant(NAME, "reverseExtensionLimit") * extensionTicksPerInch, Constants.kCANTimeoutMs);
        extensionMotor.configClosedLoopPeakOutput(2, extensionPeakOutput, Constants.kCANTimeoutMs);
        extensionMotor.selectProfileSlot(extensionPIDSlot, 0); // uses the system slot2 configuration for extension control

        double angularPeakOutput = 1;
        angleMotorMain.configPeakOutputForward(angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configPeakOutputReverse(-angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configClosedLoopPeakOutput(0, angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorFollower.configPeakOutputForward(angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorFollower.configPeakOutputReverse(-angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configClosedLoopPeakOutput(0, angularPeakOutput, Constants.kCANTimeoutMs);

        angleMotorMain.configClosedloopRamp(0.2, Constants.kCANTimeoutMs);
        extensionMotor.configClosedloopRamp(0.05, Constants.kCANTimeoutMs);

        motionMagicEnabled = factory.getConstant(NAME, "motionMagicEnabled") > 0;

        allowableAngleError = factory.getConstant(NAME, "allowableAngleError") * angleTicksPerDegree;
        allowableExtensionError = factory.getConstant(NAME, "allowableExtensionError") * extensionTicksPerInch;

        if (motionMagicEnabled) {
            var motionMagicCruiseVelTicksPer100ms = factory.getConstant(NAME, "motionMagicCruiseVelocity");
            var motionMagicAccelTicksPer100msPerSecond = factory.getConstant(NAME, "motionMagicAcceleration");
            extensionMotor.configMotionCruiseVelocity(motionMagicCruiseVelTicksPer100ms, Constants.kCANTimeoutMs);
            extensionMotor.configMotionAcceleration(motionMagicAccelTicksPer100msPerSecond, Constants.kCANTimeoutMs);
        }

        if (Constants.kLoggingRobot) {
            desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Angle/desiredPosition");
            actStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Angle/actualPosition");
            armCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Angle/currentDraw");

            desiredExtensionLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Extension/desiredExtensionPosition");
            actualExtensionLogger = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Extension/actualExtensionPosition");
            extensionCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Extension/currentDraw");
        }
    }

    /**
     * Sets the desired angle and extension state of the elevator
     *
     * @param elevatorAngleState     - the angle of the elevator
     * @param elevatorExtensionState - how far the elevator is extended
     * @see this#setDesiredAngleState(ANGLE_STATE)
     * @see this#setDesiredExtensionState(EXTENSION_STATE)
     */
    public void setDesiredState(ANGLE_STATE elevatorAngleState, EXTENSION_STATE elevatorExtensionState) {
        setDesiredAngleState(elevatorAngleState);
        setDesiredExtensionState(elevatorExtensionState);
    }

    /**
     * Sets the desired angle and extension state of the elevator
     *
     * @param desiredAngleState - Desired state for the angle of the elevator
     */
    public void setDesiredAngleState(ANGLE_STATE desiredAngleState) {
        this.desiredAngleState = desiredAngleState;
        angleOutputsChanged = true;
    }

    /**
     * Sets the desired angle and extension state of the elevator
     *
     * @param desiredExtensionState - Desired state for the extension of the elevator
     */
    public void setDesiredExtensionState(EXTENSION_STATE desiredExtensionState) {
        this.desiredExtensionState = desiredExtensionState;
        extensionOutputsChanged = true;
    }

    /**
     * Returns the desired angle of the elevator
     *
     * @return desiredAngleState
     */
    public ANGLE_STATE getDesiredAngleState() {
        return this.desiredAngleState;
    }

    /**
     * Returns the desired extension of the elevator
     *
     * @return desiredExtensionState
     */
    public EXTENSION_STATE getDesiredExtensionState() {
        return this.desiredExtensionState;
    }

    /**
     * Returns the actual position of the angle motors
     *
     * @return actual angle position
     */
    public double getActualAngleTicks() {
        return actualAngleTicks;
    }

    /**
     * Returns the actual position of the extension motors
     *
     * @return actual extension position
     */
    public double getActualExtensionTicks() {
        return actualExtensionTicks;
    }

    /**
     * Returns the error of the angular position of the arm
     */
    public double getAngleError() {
        return getActualAngleTicks() - desiredAngleTicks;
    }

    /**
     * Returns the error of the extension position of the arm
     */
    public double getExtensionError() {
        return getActualExtensionTicks() - desiredExtensionTicks;
    }

    /**
     * Returns the allowable angle error of the arm
     *
     * @return allowable angle error
     */
    public double getAllowableAngleError() {
        return allowableAngleError;
    }

    /**
     * Returns the allowable extension error of the arm
     *
     * @return allowable extension error
     */
    public double getAllowableExtensionError() {
        return allowableExtensionError;
    }

    /**
     * Reads extension and angle motor positions and their corresponding velocities
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        actualAngleTicks = angleMotorMain.getSelectedSensorPosition(0);
        actualAngleVel = angleMotorMain.getSelectedSensorVelocity(0);

        actualExtensionTicks = extensionMotor.getSelectedSensorPosition(0); // not slot id
        actualExtensionVel = extensionMotor.getSelectedSensorVelocity(0); // not slot id

        if (robotState.actualElevatorAngleState != desiredAngleState && armAtTarget()) {
            angleOutputsChanged = true;
            robotState.actualElevatorAngleState = desiredAngleState;
        }

        if (robotState.actualElevatorExtensionState != desiredExtensionState && elevatorAtTarget()) {
            extensionOutputsChanged = true;
            robotState.actualElevatorExtensionState = desiredExtensionState;
        }

        if(RobotBase.isSimulation()){
            double elevatorLength = kElevatorMinLength +
                    (actualExtensionTicks / maxExtension * (kElevatorMaxLength - kElevatorMinLength));

            simArm.setLength(elevatorLength);
            simArm.setAngle(actualAngleTicks / angleTicksPerDegree);
            SmartDashboard.putData("Elevator Mech 2D", mechCanvas);
        }

        if (Constants.kLoggingRobot) {
            ((DoubleLogEntry) desStatesLogger).append(desiredAngleTicks);
            ((DoubleLogEntry) actStatesLogger).append(actualAngleTicks);
            armCurrentDraw.append(angleMotorMain.getOutputCurrent());

            desiredExtensionLogger.append(desiredExtensionTicks);
            actualExtensionLogger.append(actualExtensionTicks);
            extensionCurrentDraw.append(extensionMotor.getOutputCurrent());
        }
    }

    /**
     * Writes outputs to extension and angle motors
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (angleOutputsChanged) {
            angleOutputsChanged = false;

            int slot = robotState.actualElevatorAngleState == desiredAngleState && armAtTarget() ? lockedArmSlot : movingArmSlot;

            double anglePos = 0;
            switch (desiredAngleState) {
                case STOW -> {
                    anglePos = (stowPos);
                }
                case COLLECT -> {
                    anglePos = collectPos;
                }
                case SCORE -> {
                    anglePos = (scorePos);
                }
                case SHELF_COLLECT -> {
                    anglePos = (shelfPos);
                }
            }
            desiredAngleTicks = anglePos;

            angleMotorMain.selectProfileSlot(slot, 0);
            angleMotorMain.set(ControlMode.Position, anglePos);
        }
        if (extensionOutputsChanged) {
            extensionOutputsChanged = false;

            ControlMode controlMode = motionMagicEnabled // && robotState.actualElevatorExtensionState != desiredExtensionState
                    ? ControlMode.MotionMagic : ControlMode.Position;

            double extension = 0;
            if (robotState.actualGameElement == Collector.GAME_ELEMENT.CONE) {
                switch (desiredExtensionState) {
                    case MAX -> extension = maxExtension;
                    case MID -> extension = midExtension;
                    case MIN -> extension = minExtension;
                    case SHELF_COLLECT -> extension = shelfExtension;
                }
            } else {
                switch (desiredExtensionState) {
                    case MAX -> extension = maxExtension;
                    case MID -> extension = midExtension;
                    case MIN -> extension = minExtension;
                    case SHELF_COLLECT -> extension = shelfExtension;
                }
            }
            desiredExtensionTicks = extension;

            System.out.println("extension set val = " + extension);
            extensionMotor.set(controlMode, extension);
        }
    }

    public boolean armAtTarget() {
        return Math.abs(desiredAngleTicks - actualAngleTicks) < getAllowableAngleError();
    }

    public boolean elevatorAtTarget() {
        return Math.abs(desiredExtensionTicks - actualExtensionTicks) < getAllowableExtensionError();
    }

    @Override
    public void zeroSensors() {
        angleMotorMain.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        angleMotorMain.selectProfileSlot(movingArmSlot, 0);
        setBraking(false);
    }

    public void setBraking(boolean braking) {
        angleMotorMain.setNeutralMode(braking ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {
    }

    /**
     * Tests the elevator, returns true if tests passed
     *
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        return false;
    }

    /**
     * Base enums
     **/
    public enum ANGLE_STATE {
        STOW(stowPos),
        COLLECT(collectPos),
        SCORE(scorePos),
        SHELF_COLLECT(shelfPos);

        private final double pos;

        ANGLE_STATE(double pos) {
            this.pos = pos;
        }

        public double getPos() {
            return pos;
        }

    }

    public enum EXTENSION_STATE {
        MIN(minExtension),
        MID(midExtension),
        MAX(maxExtension),
        SHELF_COLLECT(shelfExtension);

        private final double extension;

        EXTENSION_STATE(double extension) {
            this.extension = extension;
        }

        public double getExtension() {
            return extension;
        }
    }
}
