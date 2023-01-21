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
    private double desiredOutput;
    private double actualOutput;
    private double desiredAnglePosition;
    private double actualAnglePosition;

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

    }

    @Override
    public void readFromHardware() {

    }

    @Override
    public void writeToHardware() {}


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
    public enum STATE {
        STOP,
    }

    public enum ELEVATOR_ANGLE {
        COLLECT,
        STOW,
        SCORE,
    }

    public enum ELEVATOR_EXTENSION {
        MIN,
        MID,
        MAX,
    }

}

