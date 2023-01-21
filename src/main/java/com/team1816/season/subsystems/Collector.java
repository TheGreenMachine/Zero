package com.team1816.season.subsystems;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

public class Collector extends Subsystem {

    private static boolean isCube;

    private static boolean isHolding;

    private final ISolenoid collectorSolenoid;

    private final IGreenMotor intakeMotor;

    private STATE desiredState = STATE.STOP;

    private boolean outputsChanged = false;

    private static final String NAME = "collector";

    public Collector (Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        collectorSolenoid = factory.getSolenoid(NAME, "collectorSolenoid");
        intakeMotor = factory.getMotor(NAME, "intakeMotor");

    }

    public void setDesiredState(STATE state){
        if (desiredState != state){
            desiredState = state;
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
                switch (desiredState)
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

    public enum STATE {
        STOP,
        COLLECT_CONE,
        COLLECT_CUBE,
    }
}
