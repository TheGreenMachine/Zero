package com.team1816.season.subsystems;

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

    /** components */
    private final IGreenMotor angleMotor1;
    private final IGreenMotor angleMotor2;
    private final IGreenMotor elevatorMotor;

    /** states */
    double angleState;

    private double actualExtensionPosition;
    private double actualAnglePosition;
    private  ANGLE_STATE desiredAnglePosition = ANGLE_STATE.STOW;
    private  EXTENSION_STATE desiredExtensionPosition = EXTENSION_STATE.MIN;


    private boolean outputsChanged;
    private final double ALLOWABLE_ERROR;

    public Elevator(String name, Infrastructure inf, RobotState rs, ISolenoid elevatorSolenoid, IGreenMotor angleMotor, IGreenMotor elevatorMotor, IGreenMotor angleMotor2, double allowable_error) {
        super(name, inf, rs);
        PIDSlotConfiguration config = factory.getPidSlotConfig(NAME);


        /** components */
        this.angleMotor1 = factory.getMotor(NAME,"angleMotor1");
        this.angleMotor2 = factory.getFollowerMotor(NAME,"angleMotor2", angleMotor1);
        this.elevatorMotor = factory.getMotor(NAME,"elevatorMotor");
        /** constants */
        ALLOWABLE_ERROR = config.allowableError;
        double MAX_TICKS = factory.getConstant(NAME, "maxVelTicks100ms", 0);

    }



    public void setDesiredState(ANGLE_STATE elevatorAngle, EXTENSION_STATE elevatorExtension) {

    }

    @Override
    public void readFromHardware() {

    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;

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
        COLLECT(collectAngle),
        STOW(stowAngle),
        SCORE(scoreAngle),
        private double angle;
        ANGLE_STATE(double angle) {this.angle = angle;}
        public double getAngle() {return angle;}

    }

    public enum EXTENSION_STATE {
        MIN(minExtension),
        MID(midExtension),
        MAX(maxExtension),
        private double extension;
        EXTENSION_STATE(double extension) {this.extension = extension;}
        public double getExtension() {return extension}
    }

}

