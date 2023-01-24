package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

public class Collector extends Subsystem {

    private static boolean isCube;

    private static boolean isHolding;

    private final ISolenoid collectorPiston;

    private boolean armUp;

    private final IGreenMotor intakeMotor;

    private double intakeVel;

    private PIVOT_STATE desiredPivotState = PIVOT_STATE.DOWN;

    private COLLECTOR_STATE desiredCollectorState = COLLECTOR_STATE.STOP;

    private boolean outputsChanged = false;

    private static final String NAME = "collector";

    public Collector (Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        collectorPiston = factory.getSolenoid(NAME, "collectorSolenoid");
        intakeMotor = factory.getMotor(NAME, "intakeMotor");
    }

    public void setDesiredState(PIVOT_STATE pivotState, COLLECTOR_STATE collectorState){
        if (desiredPivotState != pivotState) {
            desiredPivotState = pivotState;
            outputsChanged = true;
        }
        if (desiredCollectorState != collectorState) {
            desiredCollectorState = collectorState;
            outputsChanged = true;
        }
    }
    @Override
    public void readFromHardware() {
        intakeMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
<<<<<<< Updated upstream
            armUp = false;
                switch (desiredPivotState) {
                    case UP:
                        break;
                    case DOWN:
                        break;
                }
                switch (desiredCollectorState) {
                    case STOP:
                        break;
                    case COLLECT:
                        break;
                    case FLUSH:
                        break;
=======
                switch (desiredState) {
>>>>>>> Stashed changes
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

    //for cube collecting, the "motor locked" state so we don't pop the cube will have
    //to be the same as stopping? a hold action and then when its not going it stops? we can't
    //rotate after getting a cube anyways
    public enum PIVOT_STATE {
        UP,
        DOWN,
    }

    public enum COLLECTOR_STATE {
        STOP,
        COLLECT,
        FLUSH,
    }
}
