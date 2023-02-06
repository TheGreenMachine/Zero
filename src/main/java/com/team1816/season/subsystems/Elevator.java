package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

import javax.inject.Inject;

public class Elevator extends Subsystem {


    private static final String NAME = "elevator";

    /** components */
    private final IGreenMotor angleMotorMain;
    private final IGreenMotor angleMotorFollower;
    private final IGreenMotor extensionMotor;

    private static double stowAngle;
    private static double collectAngle;
    private static double scoreAngle;
    private static double minExtension;
    private static double midExtension;
    private static double maxExtension;
    private double actualExtensionPosition;
    private double actualAnglePosition;
    private ANGLE_STATE desiredAnglePosition = ANGLE_STATE.STOW;
    private EXTENSION_STATE desiredExtensionPosition = EXTENSION_STATE.MIN;
    private boolean outputsChanged;

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    @Inject
    public Elevator(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);

        //components
        this.angleMotorMain = factory.getMotor(NAME,"angleMotorMain");
        this.angleMotorFollower = factory.getFollowerMotor(NAME,"angleMotorFollower", angleMotorMain);
        this.extensionMotor = factory.getMotor(NAME,"extensionMotor");

        //constants
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

    }

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

