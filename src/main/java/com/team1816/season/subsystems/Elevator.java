package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.DigitalInput;

import javax.inject.Inject;
import javax.inject.Singleton;

@Singleton
public class Elevator extends Subsystem {
    private static final String NAME = "elevator";

    /**
     * Components
     */
    private final IGreenMotor angleMotorMain;
    private final IGreenMotor angleMotorFollower;
    private final IGreenMotor extensionMotor;
    private final DigitalInput hallEffect;

    /**
     * Properties
     */
    private static double stowAngle;
    private static double collectAngle;
    private static double scoreAngle;
    private static double minExtension;
    private static double midExtension;
    private static double maxExtension;
    private static double maxAngularVelocity; // rad/s
    private static double maxAngularAcceleration; // rad/s^2
    private static double maxExtendedAngularAcceleration; // rad/s^2
    private static double maxExtensionVelocity; // m/s
    private static double maxExtensionAcceleration; // m/s^2


    /**
     * States
     */
    private double actualExtensionPosition;
    private double actualAnglePosition;
    private double actualAngleThetaDegrees;
    private double actualExtensionMeters;
    private double actualAngleVel;
    private double actualExtensionVel;
    private ANGLE_STATE desiredAnglePosition = ANGLE_STATE.STOW;
    private EXTENSION_STATE desiredExtensionPosition = EXTENSION_STATE.MIN;
    private boolean outputsChanged;
    private boolean hallEffectTriggered;
    private double hallEffectTriggerValue;


    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    @Inject
    public Elevator(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        //components
        this.angleMotorMain = factory.getMotor(NAME,"angleMotorMain");
        this.angleMotorFollower = factory.getFollowerMotor(NAME,"angleMotorFollower", angleMotorMain);
        this.extensionMotor = factory.getMotor(NAME,"extensionMotor");
        this.hallEffect = new DigitalInput(0);

        double peakOutput = 0.60;
        angleMotorMain.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configClosedLoopPeakOutput(0, peakOutput, Constants.kCANTimeoutMs);
        angleMotorFollower.configPeakOutputForward(peakOutput, Constants.kCANTimeoutMs);
        angleMotorFollower.configPeakOutputReverse(-peakOutput, Constants.kCANTimeoutMs);
        angleMotorMain.configClosedLoopPeakOutput(0, peakOutput, Constants.kCANTimeoutMs);

        // constants
        stowAngle = factory.getConstant(NAME,"stowAnglePosition");
        collectAngle = factory.getConstant(NAME, "collectAnglePosition");
        scoreAngle = factory.getConstant(NAME, "scoreAnglePosition");
        minExtension = factory.getConstant(NAME, "minExtensionPosition");
        midExtension = factory.getConstant(NAME, "midExtensionPosition");
        maxExtension = factory.getConstant(NAME, "maxExtensionPosition");

        maxAngularVelocity = factory.getConstant(NAME, "maxAngularVelocity");
        maxAngularAcceleration = factory.getConstant(NAME, "maxAngularAcceleration");
        maxExtensionAcceleration = factory.getConstant(NAME, "maxExtendedAngularAcceleration");
        maxExtensionVelocity = factory.getConstant(NAME, "maxExtensionVelocity");
        maxExtensionAcceleration = factory.getConstant(NAME, "maxExtensionAcceleration");
    }

    /**
     * Sets the desired angle and extension state of the elevator
     * @param elevatorAngleState - the angle of the elevator
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
     * @param desiredAngleState - Desired state for the angle of the elevator
     */
    public void setDesiredAngleState(ANGLE_STATE desiredAngleState) {
        if (desiredAnglePosition != desiredAngleState) {
            desiredAnglePosition = desiredAngleState;
            outputsChanged = true;
        }
    }

    /**
     * Sets the desired angle and extension state of the elevator
     * @param desiredExtensionState - Desired state for the extension of the elevator
     */
    public void setDesiredExtensionState(EXTENSION_STATE desiredExtensionState) {
        if (desiredExtensionPosition != desiredExtensionState) {
            desiredExtensionPosition = desiredExtensionState;
            outputsChanged = true;
        }
    }

    /**
     * Reads extension and angle motor positions and their corresponding velocities
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        if(actualAngleVel != angleMotorMain.getSelectedSensorVelocity(0)){ //TODO we need a method of motor calibration
            System.out.println(angleMotorMain.getSelectedSensorVelocity(0));
        }
        actualAnglePosition = angleMotorMain.getSelectedSensorPosition(0);
        actualAngleVel = angleMotorMain.getSelectedSensorVelocity(0);

        actualExtensionPosition = extensionMotor.getSelectedSensorPosition(0);
        actualExtensionVel = extensionMotor.getSelectedSensorVelocity(0);

        if(hallEffectTriggered == hallEffect.get()){
            hallEffectTriggerValue = actualAnglePosition;
            System.out.println(hallEffectTriggerValue);
        }
        hallEffectTriggered = !hallEffect.get();


    }

    /**
     * Writes outputs to extension and angle motors
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredExtensionPosition){
                case MAX:
                    extensionMotor.set(ControlMode.Position, (maxExtension));
                    break;
                case MID:
                    extensionMotor.set(ControlMode.Position, (midExtension));
                    break;
                case MIN:
                    extensionMotor.set(ControlMode.Position, (minExtension));
                    break;

            }
            switch (desiredAnglePosition) {
                case STOW:
                    angleMotorMain.set(ControlMode.Position, (stowAngle));
                    break;
                case COLLECT:
                    angleMotorMain.set(ControlMode.Position, (collectAngle));
                    break;
                case SCORE:
                    angleMotorMain.set(ControlMode.Position, (scoreAngle));
                    break;
            }

        }
    }

    // TODO later
    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean testSubsystem() {
        return false;
    }

    /** Base enums **/
    public enum ANGLE_STATE {
        STOW(stowAngle),
        COLLECT(collectAngle),
        SCORE(scoreAngle);

        private final double angle;
        ANGLE_STATE(double angle) {this.angle = angle;}
        public double getAngle() {return angle;}

    }

    public enum EXTENSION_STATE {
        MIN(minExtension),
        MID(midExtension),
        MAX(maxExtension);

        private final double extension;
        EXTENSION_STATE(double extension) {this.extension = extension;}
        public double getExtension() {return extension;}
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
        private double endAccelerationPhase;
        private double endVelocityPhase;
        private double endDecelerationPhase;

        /**
         * Polar properties
         */
        private double a; // polar coefficient
        private double b; // polar coefficient
        private double c; // polar coefficient

        /**
         * Initializes a setpoint feeder
         * @param f profile constraints
         */
        public SetPointFeeder(FeederConstraints f) {
            feederConstraints = f; // initializes constraints
            // calculates relative timestamps to 0
            double distance = (thetaFinal - thetaInitial);
            double accelerationTime = feederConstraints.maxAngularVelocity / feederConstraints.maxAngularAcceleration;
            double decelerationTime = feederConstraints.maxAngularVelocity / feederConstraints.maxExtendedAngularAcceleration;
            double maxDist = distance - accelerationTime * accelerationTime * feederConstraints.maxAngularAcceleration / 2 - decelerationTime * decelerationTime * feederConstraints.maxExtendedAngularAcceleration / 2;

            if (maxDist < 0) {
                accelerationTime = Math.sqrt(distance / feederConstraints.maxAngularAcceleration);
                decelerationTime = Math.sqrt(distance / feederConstraints.maxExtendedAngularAcceleration);
                maxDist = 0;
            }

            endAccelerationPhase = accelerationTime;
            endVelocityPhase = endAccelerationPhase + maxDist / feederConstraints.maxAngularVelocity;
            endDecelerationPhase = endVelocityPhase + decelerationTime;

            a = rFinal*Math.sin(thetaFinal) - rInitial*Math.sin(thetaInitial);
            b = -1 * (rFinal*Math.cos(thetaFinal) - rInitial*Math.cos(thetaInitial));
            c = -1 * rInitial*Math.sin(thetaInitial) * (rFinal*Math.cos(thetaFinal) - rInitial*Math.cos(thetaInitial));
        }

        /**
         * Initializes a full setpoint feeder with initial and final constraints
         * @param f profile constraints
         * @param rInitial initial extension
         * @param thetaInitial initial angle
         * @param rFinal final extension
         * @param thetaFinal final angle
         */
        public SetPointFeeder(FeederConstraints f, double rInitial, double thetaInitial, double rFinal, double thetaFinal) {
            this.rInitial = rInitial;
            this.thetaInitial = thetaInitial;
            this.rFinal = rFinal;
            this.thetaFinal = thetaFinal;

            new SetPointFeeder(f);
        }

        /**
         * Sets the starting timestamp of the set point feeder
         * @param timestamp
         */
        public void start(double timestamp) {
            startTimestamp = timestamp;
        }

        /**
         * Returns the profiled angular polar component based on constraints
         * @param timestamp current timestamp
         * @return angle
         */
        public double getAngle(double timestamp) {
            double t = timestamp - startTimestamp;
            if (t <= endAccelerationPhase) {
                return t * t * feederConstraints.maxAngularVelocity / 2;
            } else if (t <= endVelocityPhase) {
                return endAccelerationPhase * endAccelerationPhase * feederConstraints.maxAngularAcceleration / 2 + feederConstraints.maxAngularVelocity * (t - endVelocityPhase);
            } else if (t <= endDecelerationPhase) {
                double timeRemaining = endDecelerationPhase - t;
                return thetaFinal - (timeRemaining * timeRemaining * feederConstraints.maxExtendedAngularAcceleration / 2);
            } else {
                return thetaFinal;
            }
        }

        /**
         * Feeds positional setpoints based on the current timestamp
         * @param timestamp current timestamp
         * @return setpoints
         */
        public double[] get(double timestamp) {
            double angle = getAngle(timestamp);
            double extension = c / (a * Math.cos(angle) + b * Math.sin(angle));
            return new double[] {angle, extension};
        }
    }

    /**
     * Class for SetPointFeeder constraints
     */
    public static class FeederConstraints {
        public double maxAngularVelocity; // rad/s
        public double maxExtendedAngularAcceleration; // rad/s^2
        public double maxAngularAcceleration; // rad/s^2
        public double maxExtensionVelocity; // m/s
        public double maxExtensionAcceleration; // m/s^2

        public FeederConstraints(double maxAngularVelocity, double maxExtendedAngularAcceleration, double maxAngularAcceleration, double maxExtensionVelocity, double maxExtensionAcceleration) {
            this.maxAngularVelocity = maxAngularVelocity;
            this.maxExtendedAngularAcceleration = maxExtendedAngularAcceleration;
            this.maxAngularAcceleration = maxAngularAcceleration;
            this.maxExtensionVelocity = maxExtensionVelocity;
            this.maxExtensionAcceleration = maxExtensionAcceleration;
        }
    }
}
