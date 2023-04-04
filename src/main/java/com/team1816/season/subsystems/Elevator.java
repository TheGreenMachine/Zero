package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

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
    public static final double stowPos = factory.getConstant(NAME, "stowAnglePosition");
    public static final double collectPos = factory.getConstant(NAME, "collectAnglePosition");
    public static final double scorePos = factory.getConstant(NAME, "scoreAnglePosition");

    public static final double shelfPos = factory.getConstant(NAME, "shelfAnglePosition");
    public static final double minExtension = factory.getConstant(NAME, "minExtensionPosition");
    public static final double midExtension = factory.getConstant(NAME, "midExtensionPosition");
    public static final double maxExtension = factory.getConstant(NAME, "maxExtensionPosition");
    public static final double shelfExtension = factory.getConstant(NAME, "shelfExtensionPosition");

    public static final double cubeExtensionMaxOffset = factory.getConstant("cubeExtensionMaxOffset", 10000);
    public static final double cubeExtensionMidOffset = factory.getConstant("cubeExtensionMidOffset", 5000);

    private AsyncTimer colPosTimer;
    private static double allowableAngleError;
    private static double allowableExtensionError;

    private static double maxAngularVelocity; // rad/s
    private static double maxAngularAcceleration; // rad/s^2
    private static double maxExtendedAngularAcceleration; // rad/s^2
    private static double maxExtensionVelocity; // m/s
    private static double maxExtensionAcceleration; // m/s^2
    private final int movingArmSlot = 0;
    private final int lockedArmSlot = 1;
    private final int extensionPIDSlot = 2;

    private static double angleQuarterPPR;
    private static double extensionPPR;
    private boolean motionMagicEnabled = false;

    /**
     * States
     */
    private double actualExtensionPosition = 0;
    private double actualAnglePosition = 0;
    private double actualAngleThetaDegrees;
    private double actualExtensionMeters;
    private double actualAngleVel;
    private double actualExtensionVel;
    private ANGLE_STATE desiredAngleState = ANGLE_STATE.STOW;
    private EXTENSION_STATE desiredExtensionState = EXTENSION_STATE.MIN;
    private double angleFeedForward;
    private double extensionFeedForward;

    private boolean angleOutputsChanged;
    private boolean extensionOutputsChanged;
    private boolean hallEffectTriggered; // not using this rn - turn on robot with arm all the way down for ~20 secs
    private double zeroingHallEffectTriggerValue;

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
        extensionMotor.configForwardSoftLimitThreshold(factory.getConstant(NAME, "forwardExtensionLimit"), Constants.kCANTimeoutMs);
        extensionMotor.configReverseSoftLimitThreshold(factory.getConstant(NAME, "reverseExtensionLimit"), Constants.kCANTimeoutMs);
        extensionMotor.configClosedLoopPeakOutput(1, extensionPeakOutput, Constants.kCANTimeoutMs);
        extensionMotor.selectProfileSlot(extensionPIDSlot, 0); // uses the system slot1 configuration for extension control

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
        extensionPPR = factory.getConstant(NAME, "extensionPPR");
        angleQuarterPPR = factory.getConstant(NAME, "angleQuarterPPR");

        allowableAngleError = factory.getPidSlotConfig(NAME, "slot0").allowableError;
        allowableExtensionError = factory.getPidSlotConfig(NAME, "slot2").allowableError;

        colPosTimer = new AsyncTimer(
                1,
                () -> {
                    angleMotorMain.set(ControlMode.Position, collectPos);
                },
                () -> {
                    // set it to go down until it hits rubber then just fight against the spring to stay down
                    // that way we don't need to be dead-on for the collect pos
                    angleMotorMain.set(ControlMode.PercentOutput, -0.06);   //(start -.08)i can go up to -0.1 if collecting too high
                    GreenLogger.log("running collector into rubber w/ %out");
                }
        );

        if (motionMagicEnabled) {
            var motionMagicCruiseVelTicksPer100ms = factory.getConstant(NAME, "motionMagicCruiseVelocity");
            var motionMagicAccelTicksPer100msPerSecond = factory.getConstant(NAME, "motionMagicAcceleration");
            extensionMotor.configMotionSCurveStrength(2, Constants.kCANTimeoutMs);
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
    public double getActualAnglePosition() {
        return actualAnglePosition;
    }

    /**
     * Returns the actual position of the extension motors
     *
     * @return actual extension position
     */
    public double getActualExtensionPosition() {
        return actualExtensionPosition;
    }

    /**
     * Returns the error of the angular position of the arm
     */
    public double getAngleError() {
        return getActualAnglePosition() - getDesiredAngleState().getPos();
    }

    /**
     * Returns the error of the extension position of the arm
     */
    public double getExtensionError() {
        return getActualExtensionPosition() - getDesiredExtensionState().getExtension();
    }

    /**
     * Returns the allowable angle error of the arm
     *
     * @return allowable angle error
     */
    public double getAllowableAngleError() {
        return allowableAngleError * 8;
    }

    /**
     * Returns the allowable extension error of the arm
     *
     * @return allowable extension error
     */
    public double getAllowableExtensionError() {
        return allowableExtensionError * 16;
    }

    /**
     * Reads extension and angle motor positions and their corresponding velocities
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        actualAnglePosition = angleMotorMain.getSelectedSensorPosition(0);
        actualAngleVel = angleMotorMain.getSelectedSensorVelocity(0);

        actualExtensionPosition = extensionMotor.getSelectedSensorPosition(0); // not slot id
        actualExtensionVel = extensionMotor.getSelectedSensorVelocity(0); // not slot id
        if (robotState.actualElevatorAngleState != desiredAngleState && armAtTarget()) {
            angleOutputsChanged = true;
            robotState.actualElevatorAngleState = desiredAngleState;
        }

        if (robotState.actualElevatorExtensionState != desiredExtensionState && elevatorAtTarget()) {
            extensionOutputsChanged = true;
            robotState.actualElevatorExtensionState = desiredExtensionState;
        }

        if (Constants.kLoggingRobot) {
            ((DoubleLogEntry) desStatesLogger).append(getDesiredAngleState().pos);
            ((DoubleLogEntry) actStatesLogger).append(actualAnglePosition);
            armCurrentDraw.append(angleMotorMain.getOutputCurrent());

            desiredExtensionLogger.append(getDesiredExtensionState().extension);
            actualExtensionLogger.append(actualExtensionPosition);
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
            int slot = robotState.actualElevatorAngleState == desiredAngleState ? lockedArmSlot : movingArmSlot;

            switch (desiredAngleState) {
                case STOW -> {
                    angleMotorMain.selectProfileSlot(slot, 0);
                    angleMotorMain.set(ControlMode.Position, (stowPos));
                }
                case COLLECT -> {
                    angleMotorMain.selectProfileSlot(slot, 0);
                    colPosTimer.update();
                    if (!colPosTimer.isCompleted()) {
                        angleOutputsChanged = true;
                    } else {
                        colPosTimer.reset();
                    }
                }
                case SCORE -> {
                    angleMotorMain.selectProfileSlot(slot, 0);
                    angleMotorMain.set(ControlMode.Position, (scorePos));
                }
                case SHELF_COLLECT -> {
                    angleMotorMain.selectProfileSlot(slot, 0);
//                    if(robotState.actualGameElement == Collector.GAME_ELEMENT.CUBE){
//                        angleMotorMain.set(ControlMode.Position, shelfPos - 5000); // TODO WATCH ME
//                    } else {
                    angleMotorMain.set(ControlMode.Position, (shelfPos));
//                    }
                }
            }
        }
        if (extensionOutputsChanged) {
            extensionOutputsChanged = false;
            ControlMode controlMode = motionMagicEnabled && robotState.actualElevatorExtensionState != desiredExtensionState
                    ? ControlMode.MotionMagic : ControlMode.Position;

            if (robotState.actualGameElement == Collector.GAME_ELEMENT.CONE) {
                switch (desiredExtensionState) {
                    case MAX -> extensionMotor.set(controlMode, (maxExtension));
                    case MID -> extensionMotor.set(controlMode, (midExtension));
                    case MIN -> extensionMotor.set(controlMode, (minExtension));
                    case SHELF_COLLECT -> extensionMotor.set(controlMode, (shelfExtension));
                }
            } else {
                switch (desiredExtensionState) {
                    case MAX -> extensionMotor.set(controlMode, (maxExtension + cubeExtensionMaxOffset));
                    case MID -> extensionMotor.set(controlMode, (midExtension + cubeExtensionMidOffset));
                    case MIN -> extensionMotor.set(controlMode, (minExtension));
                    case SHELF_COLLECT -> extensionMotor.set(controlMode, (shelfExtension + cubeExtensionMidOffset));
                }
            }
        }
    }

    public boolean armAtTarget() {
        return Math.abs(desiredAngleState.getPos() - actualAnglePosition) < getAllowableAngleError();
    }

    public boolean elevatorAtTarget() {
        return Math.abs(desiredExtensionState.getExtension() - actualExtensionPosition) < getAllowableExtensionError();
    }

    @Override
    public void zeroSensors() {
        angleMotorMain.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
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
