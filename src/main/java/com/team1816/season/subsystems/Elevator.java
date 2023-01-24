package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

@Singleton
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

    private double anglePos;
    private double extensionPos;
    private ANGLE_STATE desiredAnglePosition = ANGLE_STATE.STOW;
    private EXTENSION_STATE desiredExtensionPosition = EXTENSION_STATE.MIN;
    private boolean outputsChanged;
    private final double ALLOWABLE_ERROR;

    @Inject
    public Elevator(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);

        //components
        this.angleMotorMain = factory.getMotor(NAME,"angleMotorMain");
        this.angleMotorFollower = factory.getFollowerMotor(NAME,"angleMotorFollower", angleMotorMain);
        this.extensionMotor = factory.getMotor(NAME,"extensionMotor");

        //constants
        ALLOWABLE_ERROR = config.allowableError;
        // both components in this subsystem run in position mode, which means we want to know how many ticks (pulses)
        // there are per revolution of the motor shaft
        double MAX_TICKS = factory.getConstant(NAME,"encPPR", 0);
        // this part is dead on - we want set positions (that are tick values) to set the motors to
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
        // we're not looking for the motor's velocity but its position here -
        // how far out the arm has extended is proportional to what position the motor is (how many rotations it's done)
        extensionPos = extensionMotor.getSelectedSensorPosition(0);
        // we want to know at what angle it's at not how fast it's getting there
        anglePos = angleMotorMain.getSelectedSensorPosition(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredExtensionPosition) {
                //all are using ticks to determine their position, called from yaml constants
                // you are setting the motors to velocity mode not position -
                // rn this will make the extension arm extend to the point that it breaks!
                case MAX -> extensionMotor.set(ControlMode.Velocity, (maxExtension));
                case MID -> extensionMotor.set(ControlMode.Velocity, (midExtension));
                case MIN -> extensionMotor.set(ControlMode.Velocity, (minExtension));
            }
            switch (desiredAnglePosition) {
                //all are using ticks to determine their position, called from yaml constants
                // same as above, except this will cause a different kind of bad event
                // (arm spinning all the way around and smashing into the ground behind robot)
                case STOW -> angleMotorMain.set(ControlMode.Velocity, (stowAngle));
                case COLLECT -> angleMotorMain.set(ControlMode.Velocity, (collectAngle));
                case SCORE -> angleMotorMain.set(ControlMode.Velocity, (scoreAngle));
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
        // it's doable to make the enum store the velocity that it corresponds to,
        // but since it's usually a hassle to do and ur currently not using it I would kinda advise against it?
        STOW(stowAngle),
        COLLECT(collectAngle),
        SCORE(scoreAngle),
        ;
        private double angle;
        ANGLE_STATE(double angle) {this.angle = angle;}
        public double getAngle() {return angle;}

    }

    public enum EXTENSION_STATE {
        // same as above :)
        MIN(minExtension),
        MID(midExtension),
        MAX(maxExtension),
        ;
        private double extension;
        EXTENSION_STATE(double extension) {this.extension = extension;}
        public double getExtension() {return extension;}
    }

}

