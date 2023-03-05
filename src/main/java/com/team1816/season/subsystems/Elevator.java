package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

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
    private final DigitalInput zeroingHallEffect;

    /**
     * Properties
     */
    private static double stowAngle;
    private static double collectAngle;
    private static double scoreAngle;
    private static double scoreDipAngle;
    private static double minExtension;
    private static double midExtension;
    private static double maxExtension;

    private static double allowableAngleError;
    private static double allowableExtensionError;

    private static double maxAngularVelocity; // rad/s
    private static double maxAngularAcceleration; // rad/s^2
    private static double maxExtendedAngularAcceleration; // rad/s^2
    private static double maxExtensionVelocity; // m/s
    private static double maxExtensionAcceleration; // m/s^2

    private int anglePIDSlot = 0;
    private int extensionPIDSlot = 1;

    private static double angleQuarterPPR;
    private static double extensionPPR;

    private static boolean usingSetpointFeeder = true;
    private SetPointFeeder setPointFeeder;
    private boolean feederStarted = false;


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

    private boolean angleOutputsChanged;
    private boolean extensionOutputsChanged;
    private boolean hallEffectTriggered;
    private double zeroingHallEffectTriggerValue;


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
        this.zeroingHallEffect = new DigitalInput(0);

        double angularPeakOutput = 0.80;
        angleMotorMain.configPeakOutputForward(angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configPeakOutputReverse(-angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configClosedLoopPeakOutput(0, angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorFollower.configPeakOutputForward(angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorFollower.configPeakOutputReverse(-angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configClosedLoopPeakOutput(0, angularPeakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.selectProfileSlot(anglePIDSlot, 0); // uses the slot0 configuration for angular control

        double extensionPeakOutput = 0.80;
        extensionMotor.configPeakOutputForward(extensionPeakOutput, Constants.kCANTimeoutMs);
        extensionMotor.configPeakOutputReverse(-extensionPeakOutput, Constants.kCANTimeoutMs);
        extensionMotor.configForwardSoftLimitEnable(true, Constants.kCANTimeoutMs);
        extensionMotor.configReverseSoftLimitEnable(true, Constants.kCANTimeoutMs);
        extensionMotor.configForwardSoftLimitThreshold(factory.getConstant(NAME, "forwardExtensionLimit"), Constants.kCANTimeoutMs);
        extensionMotor.configReverseSoftLimitThreshold(factory.getConstant(NAME, "reverseExtensionLimit"), Constants.kCANTimeoutMs);
        extensionMotor.configClosedLoopPeakOutput(1, extensionPeakOutput, Constants.kCANTimeoutMs);
        extensionMotor.selectProfileSlot(extensionPIDSlot, 0); // uses the slot1 configuration for extension control

        // constants
        stowAngle = factory.getConstant(NAME, "stowAnglePosition");
        collectAngle = factory.getConstant(NAME, "collectAnglePosition");
        scoreAngle = factory.getConstant(NAME, "scoreAnglePosition");
        scoreDipAngle = factory.getConstant(NAME, "scoreDipAnglePosition");
        minExtension = factory.getConstant(NAME, "minExtensionPosition");
        midExtension = factory.getConstant(NAME, "midExtensionPosition");
        maxExtension = factory.getConstant(NAME, "maxExtensionPosition");

        allowableAngleError = factory.getPidSlotConfig(NAME, "slot0").allowableError;
        allowableExtensionError = factory.getPidSlotConfig(NAME, "slot1").allowableError;

        maxAngularVelocity = factory.getConstant(NAME, "maxAngularVelocity");
        maxAngularAcceleration = factory.getConstant(NAME, "maxAngularAcceleration");
        maxExtendedAngularAcceleration = factory.getConstant(NAME, "maxExtendedAngularAcceleration");
        maxExtensionVelocity = factory.getConstant(NAME, "maxExtensionVelocity");
        maxExtensionAcceleration = factory.getConstant(NAME, "maxExtensionAcceleration");
    }

    /**
     * Sets the desired angle and extension state of the elevator
     *
     * @param elevatorAngleState     - the angle of the elevator
     * @param elevatorExtensionState - the extension of the elevator
     * @see this#setDesiredAngleState(ANGLE_STATE)
     * @see this#setDesiredExtensionState(EXTENSION_STATE)
     */
    public void setDesiredState(ANGLE_STATE elevatorAngleState, EXTENSION_STATE elevatorExtensionState) {
        setDesiredAngleState(elevatorAngleState);
        setDesiredExtensionState(elevatorExtensionState);
        FeederConstraints feederConstraints = new FeederConstraints(
            maxAngularVelocity,
            (maxExtendedAngularAcceleration - maxAngularAcceleration) / (maxExtension - minExtension) *
                desiredExtensionState.extension,
            (maxExtendedAngularAcceleration - maxAngularAcceleration) / (maxExtension - minExtension) *
                actualExtensionPosition,
            maxExtensionVelocity,
            maxExtensionAcceleration
        );
        setPointFeeder = new SetPointFeeder(
            feederConstraints,
            actualExtensionPosition,
            actualAnglePosition,
            elevatorExtensionState.extension,
            elevatorAngleState.angle
        );
        setPointFeeder.start(Timer.getFPGATimestamp());
        feederStarted = true;
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
     * Returns the desired angle state of the elevator
     *
     * @return desiredAngleState
     */
    public ANGLE_STATE getDesiredAngleState() {
        return this.desiredAngleState;
    }

    /**
     * Returns the desired extension state of the elevator
     *
     * @return desiredExtensionState
     */
    public EXTENSION_STATE getDesiredExtensionState() {
        return this.desiredExtensionState;
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

        hallEffectTriggered = !zeroingHallEffect.get();

        if (Math.abs(desiredAngleState.getAngle() - actualAnglePosition) < allowableAngleError * 2) {
            robotState.actualElevatorAngleState = desiredAngleState;
        }
        if (Math.abs(desiredExtensionState.getExtension() - actualExtensionPosition) < allowableExtensionError * 4) {
            robotState.actualElevatorExtensionState = desiredExtensionState;
        }
    }

    /**
     * Writes outputs to extension and angle motors
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (angleOutputsChanged || extensionOutputsChanged) {
            if (usingSetpointFeeder) {
                if (feederStarted) {
                    double[] positions = setPointFeeder.get(Timer.getFPGATimestamp());
                    angleMotorMain.set(ControlMode.Position, positions[0]);
                    extensionMotor.set(ControlMode.Position, positions[1]);
                    if (setPointFeeder.ended()) {
                        feederStarted = false;
                        angleOutputsChanged = false;
                        extensionOutputsChanged = false;
                    }
                }
            } else {
                if (angleOutputsChanged) {
                    angleOutputsChanged = false;
                }
                switch (desiredAngleState) {
                    case STOW -> angleMotorMain.set(ControlMode.Position, (stowAngle));
                    case COLLECT -> angleMotorMain.set(ControlMode.Position, (collectAngle));
                    case SCORE -> angleMotorMain.set(ControlMode.Position, (scoreAngle));
                    case SCORE_DIP -> angleMotorMain.set(ControlMode.Position, (scoreDipAngle));
                }

                if (extensionOutputsChanged) {
                    extensionOutputsChanged = false;
                }
                switch (desiredExtensionState) {
                    case MAX -> extensionMotor.set(ControlMode.Position, (maxExtension));
                    case MID -> extensionMotor.set(ControlMode.Position, (midExtension));
                    case MIN -> extensionMotor.set(ControlMode.Position, (minExtension));
                }
            }
        }
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {

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
        STOW(stowAngle),
        COLLECT(collectAngle),
        SCORE(scoreAngle),
        SCORE_DIP(scoreDipAngle);

        private final double angle;

        ANGLE_STATE(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }

    }

    public enum EXTENSION_STATE {
        MIN(minExtension),
        MID(midExtension),
        MAX(maxExtension);

        private final double extension;

        EXTENSION_STATE(double extension) {
            this.extension = extension;
        }

        public double getExtension() {
            return extension;
        }
    }

    /**
     * Creates a positional set point feeder for more continuous motion
     */
    public static class SetPointFeeder {
        /**
         * Feeder properties
         */
        double startTimestamp = 0;
        public FeederConstraints feederConstraints;
        public double rInitial; // m
        public double thetaInitial; // rad where zero is the collect position
        public double rFinal; // m
        public double thetaFinal; // m

        /**
         * Profile properties
         */
        private double endRotationAccelerationPhase;
        private double endRotationVelocityPhase;
        private double endRotationDecelerationPhase;

        private double endTranslationAccelerationPhase;
        private double endTranslationVelocityPhase;
        private double endTranslationDecelerationPhase;

        private boolean rotationEnded;
        private boolean translationEnded;

        /**
         * Polar properties
         */
        private double a; // polar coefficient
        private double b; // polar coefficient
        private double c; // polar coefficient

        /**
         * Initializes a full setpoint feeder with initial and final constraints
         *
         * @param f            profile constraints
         * @param rInitial     initial extension
         * @param thetaInitial initial angle
         * @param rFinal       final extension
         * @param thetaFinal   final angle
         */
        public SetPointFeeder(FeederConstraints f, double rInitial, double thetaInitial, double rFinal, double thetaFinal) {
            this.rInitial = rInitial;
            this.thetaInitial = thetaInitial;
            this.rFinal = rFinal;
            this.thetaFinal = thetaFinal;

            feederConstraints = f; // initializes constraints
            // calculates relative timestamps to 0
            // rotation
            double rotationDistance = Math.abs(thetaFinal - thetaInitial);
            double rotationAccelerationTime = feederConstraints.maxAngularVelocity / feederConstraints.maxAngularAcceleration;
            double rotationDecelerationTime = feederConstraints.maxAngularVelocity / feederConstraints.maxAngularDeceleration;
            double maxRotationDist = rotationDistance - rotationAccelerationTime * rotationAccelerationTime * feederConstraints.maxAngularAcceleration / 2 - rotationDecelerationTime * rotationDecelerationTime * feederConstraints.maxAngularDeceleration / 2;

            if (maxRotationDist < 0) {
                rotationAccelerationTime = Math.sqrt(rotationDistance / feederConstraints.maxAngularAcceleration);
                rotationDecelerationTime = Math.sqrt(rotationDistance / feederConstraints.maxAngularDeceleration);
                maxRotationDist = 0;
            }

            endRotationAccelerationPhase = rotationAccelerationTime;
            endRotationVelocityPhase = endRotationAccelerationPhase + maxRotationDist / feederConstraints.maxAngularVelocity;
            endRotationDecelerationPhase = endRotationVelocityPhase + rotationDecelerationTime;


            if (thetaFinal - thetaInitial < 0) { // negation
                feederConstraints.maxAngularVelocity *= -1;
                feederConstraints.maxAngularAcceleration *= -1;
                feederConstraints.maxAngularDeceleration *= -1;
            }

            // translation
            double translationDistance = Math.abs(rFinal - rInitial);
            double translationAccelTime = feederConstraints.maxExtensionVelocity / feederConstraints.maxExtensionAcceleration;
            double maxTranslationDist = translationDistance - translationAccelTime * translationAccelTime * feederConstraints.maxExtensionAcceleration;

            if (maxTranslationDist < 0) {
                translationAccelTime = Math.sqrt(translationDistance / feederConstraints.maxExtensionAcceleration);
                maxTranslationDist = 0;
            }

            endTranslationAccelerationPhase = translationAccelTime;
            endTranslationVelocityPhase = endTranslationAccelerationPhase + maxTranslationDist / feederConstraints.maxExtensionVelocity;
            endTranslationDecelerationPhase = endTranslationVelocityPhase + translationAccelTime;

            if (rFinal - rInitial < 0) { // negation
                feederConstraints.maxExtensionVelocity *= -1;
                feederConstraints.maxExtensionAcceleration *= -1;
            }


            a = this.rFinal * Math.sin(this.thetaFinal) - this.rInitial * Math.sin(this.thetaInitial);
            b = -1 * (this.rFinal * Math.cos(this.thetaFinal) - this.rInitial * Math.cos(this.thetaInitial));
            c = -1 * this.rInitial * Math.sin(this.thetaInitial) * (this.rFinal * Math.cos(this.thetaFinal) - this.rInitial * Math.cos(this.thetaInitial));
        }

        /**
         * Sets the starting timestamp of the set point feeder
         *
         * @param timestamp
         */
        public void start(double timestamp) {
            startTimestamp = timestamp;
        }

        /**
         * Returns the profiled angular polar component based on constraints
         *
         * @param timestamp current timestamp
         * @return angle
         */
        public double getAngle(double timestamp) {
            double t = timestamp - startTimestamp;
            if (t <= endRotationAccelerationPhase) {
                return t * t * feederConstraints.maxAngularAcceleration / 2 + thetaInitial;
            } else if (t <= endRotationVelocityPhase) {
                return endRotationAccelerationPhase * endRotationAccelerationPhase * feederConstraints.maxAngularAcceleration / 2 +
                    feederConstraints.maxAngularVelocity * (t - endRotationAccelerationPhase) + thetaInitial;
            } else if (t <= endRotationDecelerationPhase) {
                double timeRemaining = endRotationDecelerationPhase - t;
                return thetaFinal - (timeRemaining * timeRemaining * feederConstraints.maxAngularDeceleration / 2);
            } else {
                rotationEnded = true;
                return thetaFinal;
            }
        }

        /**
         * Returns the profiled angular polar velocity component based on constraints
         *
         * @param timestamp current timestamp
         * @return angular velocity
         */
        public double getAngularVelocity(double timestamp) {
            double t = timestamp - startTimestamp;
            if (t <= endRotationAccelerationPhase) {
                return t * feederConstraints.maxAngularAcceleration;
            } else if (t <= endRotationVelocityPhase) {
                return feederConstraints.maxAngularVelocity;
            } else if (t <= endRotationDecelerationPhase) {
                double timeRemaining = endRotationDecelerationPhase - t;
                return timeRemaining * feederConstraints.maxAngularDeceleration;
            } else {
                return 0;
            }
        }

        /**
         * Returns the profiled angular polar acceleration component based on constraints
         *
         * @param timestamp current timestamp
         * @return angular acceleration
         */
        public double getAngularAcceleration(double timestamp) {
            double t = timestamp - startTimestamp;
            if (t <= endRotationAccelerationPhase) {
                return feederConstraints.maxAngularAcceleration;
            } else if (t <= endRotationVelocityPhase) {
                return 0;
            } else if (t <= endRotationDecelerationPhase) {
                return (-1) * feederConstraints.maxAngularDeceleration;
            } else {
                return 0;
            }
        }

        /**
         * Returns the profiled translational polar component based on constraints
         *
         * @param timestamp current timestamp
         * @return angle
         */
        public double getExtension(double timestamp) {
            double t = timestamp - startTimestamp;
            if (t <= endTranslationAccelerationPhase) {
                return t * t * feederConstraints.maxExtensionAcceleration / 2 + rInitial;
            } else if (t <= endTranslationVelocityPhase) {
                return endTranslationAccelerationPhase * endTranslationAccelerationPhase * feederConstraints.maxExtensionAcceleration / 2 +
                    feederConstraints.maxExtensionVelocity * (t - endTranslationAccelerationPhase) + rInitial;
            } else if (t <= endTranslationDecelerationPhase) {
                double timeRemaining = endTranslationDecelerationPhase - t;
                return rFinal - (timeRemaining * timeRemaining * feederConstraints.maxExtensionAcceleration);
            } else {
                translationEnded = true;
                return rFinal;
            }
        }

        /**
         * Returns the profiled translational polar velocity component based on constraints
         *
         * @param timestamp current timestamp
         * @return extension velocity
         */
        public double getExtensionVelocity(double timestamp) {
            double t = timestamp - startTimestamp;
            if (t <= endRotationAccelerationPhase) {
                return t * feederConstraints.maxExtensionAcceleration;
            } else if (t <= endRotationVelocityPhase) {
                return feederConstraints.maxExtensionVelocity;
            } else if (t <= endRotationDecelerationPhase) {
                double timeRemaining = endRotationDecelerationPhase - t;
                return timeRemaining * feederConstraints.maxExtensionAcceleration;
            } else {
                return 0;
            }
        }

        /**
         * Returns the profiled translational polar acceleration component based on constraints
         *
         * @param timestamp current timestamp
         * @return extension acceleration
         */
        public double getExtensionAcceleration(double timestamp) {
            double t = timestamp - startTimestamp;
            if (t <= endRotationAccelerationPhase) {
                return feederConstraints.maxExtensionAcceleration;
            } else if (t <= endRotationVelocityPhase) {
                return 0;
            } else if (t <= endRotationDecelerationPhase) {
                return (-1) * feederConstraints.maxExtensionAcceleration;
            } else {
                return 0;
            }
        }

        /**
         * Feeds profiled positional setpoints based on the current timestamp
         *
         * @param timestamp current timestamp
         * @return setpoints {rotation, translation}
         */
        public double[] get(double timestamp) {
            double angle = getAngle(timestamp);
            double extension = getExtension(timestamp);
            // double extension = c / (a * Math.cos(angle) + b * Math.sin(angle));
            return new double[]{angle, extension};
        }

        /**
         * Returns true if the feeder is complete
         *
         * @return true if the feeder is complete
         */
        public boolean ended() {
            return rotationEnded && translationEnded;
        }

        /**
         * Returns true if the angular section is complete
         */
        public boolean rotationEnded() {
            return rotationEnded;
        }

        /**
         * Returns true if the extension section is complete
         */
        public boolean extensionEnded() {
            return extensionEnded();
        }
    }

    /**
     * Class for SetPointFeeder constraints
     */
    public static class FeederConstraints {
        public double maxAngularVelocity; // rad/s
        public double maxAngularDeceleration; // rad/s^2
        public double maxAngularAcceleration; // rad/s^2
        public double maxExtensionVelocity; // m/s
        public double maxExtensionAcceleration; // m/s^2

        public FeederConstraints(double maxAngularVelocity, double maxAngularDeceleration, double maxAngularAcceleration, double maxExtensionVelocity, double maxExtensionAcceleration) {
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxAngularDeceleration = maxAngularDeceleration;
            this.maxAngularAcceleration = maxAngularAcceleration;
            this.maxExtensionVelocity = maxExtensionVelocity;
            this.maxExtensionAcceleration = maxExtensionAcceleration;
        }
    }
}