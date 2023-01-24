package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

public class Elevator extends Subsystem {
    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param name String
     * @param inf  Infrastructure
     * @param rs   RobotState
     */

    private static final String NAME = "elevator";

    //components
    private final IGreenMotor angleMotorMain;
    private final IGreenMotor angleMotorFollower;
    private final IGreenMotor extensionMotor;

    private static double stowAngle;
    private static double collectAngle;
    private static double scoreAngle;
    private static double minExtension;
    private static double midExtension;
    private static double maxExtension;
    private double angleVel;
    private double extensionVel;
    private double actualExtensionPosition;
    private double actualAnglePosition;
    private ANGLE_STATE desiredAnglePosition = ANGLE_STATE.STOW;
    private EXTENSION_STATE desiredExtensionPosition = EXTENSION_STATE.MIN;
    private boolean outputsChanged;
    private final double ALLOWABLE_ERROR;

    public Elevator(String name, Infrastructure inf, RobotState rs, ISolenoid elevatorSolenoid, IGreenMotor angleMotor, IGreenMotor elevatorMotor, IGreenMotor angleMotorFollower, double allowable_error) {
        super(name, inf, rs);
        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);

        //components
        this.angleMotorMain = factory.getMotor(NAME,"angleMotorMain");
        this.angleMotorFollower = factory.getFollowerMotor(NAME,"angleMotorFollower", angleMotorMain);
        this.extensionMotor = factory.getMotor(NAME,"extensionMotor");

        //constants
        ALLOWABLE_ERROR = config.allowableError;
        double MAX_TICKS = factory.getConstant(NAME, "maxVelTicks100ms", 0);
        stowAngle = factory.getConstant("elevator","stowPose");
        collectAngle = factory.getConstant("elevator", "collectPose");
        scoreAngle = factory.getConstant("elevator", "scorePose");
        minExtension = factory.getConstant("elevator", "minPose");
        midExtension = factory.getConstant("elevator", "midPose");
        maxExtension = factory.getConstant("elevator", "maxPose");
    }

    public void setDesiredState(ANGLE_STATE elevatorAngle, EXTENSION_STATE elevatorExtension) {
        if (desiredAnglePosition != elevatorAngle) {
            desiredAnglePosition = elevatorAngle;
            outputsChanged = true;
        }

        if (desiredExtensionPosition != elevatorExtension) {
            desiredExtensionPosition = elevatorExtension;
            outputsChanged = true;
        }
    }

    @Override
    public void readFromHardware() {
        extensionVel = extensionMotor.getSelectedSensorVelocity(0);
        angleVel = angleMotorMain.getSelectedSensorVelocity(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredExtensionPosition){
                //all are using ticks to determine their position, called from yaml constants
                case MAX:
                    extensionMotor.set(ControlMode.Velocity, (maxExtension));
                    break;
                case MID:
                    extensionMotor.set(ControlMode.Velocity, (midExtension));
                    break;
                case MIN:
                    extensionMotor.set(ControlMode.Velocity, (minExtension));
                    break;

            }
            switch (desiredAnglePosition) {
                //all are using ticks to determine their position, called from yaml constants
                case STOW:
                    angleMotorMain.set(ControlMode.Velocity, (stowAngle));
                    break;
                case COLLECT:
                    angleMotorMain.set(ControlMode.Velocity, (collectAngle));
                    break;
                case SCORE:
                    angleMotorMain.set(ControlMode.Velocity, (scoreAngle));
                    break;
            }

        }
    }


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

    /** enums */
    public enum ANGLE_STATE {
        STOW(stowAngle),
        COLLECT(collectAngle),
        SCORE(scoreAngle),
        ;
        private double angle;
        ANGLE_STATE(double angle) {this.angle = angle;}
        public double getAngle() {return angle;}

    }

    public enum EXTENSION_STATE {
        MIN(minExtension),
        MID(midExtension),
        MAX(maxExtension),
        ;
        private double extension;
        EXTENSION_STATE(double extension) {this.extension = extension;}
        public double getExtension() {return extension;}
    }

}
