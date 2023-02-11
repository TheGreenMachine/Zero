package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

import javax.inject.Inject;
import javax.inject.Singleton;

// Oh my, where's the well formatted documentation??
@Singleton
public class Collector extends Subsystem {


    //piston is for the switch from cone collecting state to
    private final ISolenoid collectorPiston;
    private final IGreenMotor intakeMotor;

    private double intakeVel;


    //this is for negating the direction for cone/cube
    private boolean negation;

    private PIVOT_STATE desiredPivotState = PIVOT_STATE.DOWN;

    private COLLECTOR_STATE desiredCollectorState = COLLECTOR_STATE.STOP;

    private boolean outputsChanged = false;

    private static final String NAME = "collector";

    @Inject
    public Collector (Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        collectorPiston = factory.getSolenoid(NAME, "collectorSolenoid");
        intakeMotor = factory.getMotor(NAME, "intakeMotor");
    }

    public void setDesiredState(boolean isCube, PIVOT_STATE pivotState, COLLECTOR_STATE collectorState){
        if (isCube) {
            negation = true;
        } else {
            negation = false;
        }
        if (desiredPivotState != pivotState) {
            desiredPivotState = pivotState;
            outputsChanged = true;
        }
        if (desiredCollectorState != collectorState) {
            desiredCollectorState = collectorState;
            outputsChanged = true;
        }
    }

    /*public void setCollect(boolean setCollect){
        System.out.println("setCollect method is called");
        if(setCollect) {
            setDesiredState(PIVOT_STATE.DOWN, COLLECTOR_STATE.COLLECT);
            System.out.println("desired state is set to true!");
            outputsChanged = true;
        } else {
            setDesiredState(PIVOT_STATE.DOWN, COLLECTOR_STATE.STOP);
            outputsChanged = true;
        }
    }

    public void setEject(boolean setEject){
        System.out.println("setEject method is called");
        if(setEject) {
            setDesiredState(PIVOT_STATE.DOWN, COLLECTOR_STATE.EJECT);
            System.out.println("desired state is set to true!");
            outputsChanged = true;
        } else {
            setDesiredState(PIVOT_STATE.DOWN, COLLECTOR_STATE.STOP);
            outputsChanged = true;
        }
    }*/


    @Override
    public void readFromHardware() {
        intakeVel = intakeMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public void writeToHardware() {

        if (outputsChanged) {
            outputsChanged = false;
            if(negation) {
                switch (desiredPivotState) {
                    case UP:
                        collectorPiston.set(false);
                        break;
                    case DOWN:
                        collectorPiston.set(true);
                        break;
                }
                switch (desiredCollectorState) {
                    case STOP:
                        intakeMotor.set(ControlMode.Velocity, 0);
                        collectorPiston.set(false);
                        break;
                    case COLLECT:
                        intakeMotor.set(ControlMode.Velocity, -(factory.getConstant(NAME, ("collecting"))));
                        break;
                    case EJECT:
                        intakeMotor.set(ControlMode.Velocity, -(factory.getConstant(NAME, ("ejecting"))));
                        break;
                }
            } else {
                switch (desiredPivotState) {
                    case UP:
                        collectorPiston.set(false);
                        break;
                    case DOWN:
                        collectorPiston.set(true);
                        break;
                }
                switch (desiredCollectorState) {
                    case STOP:
                        intakeMotor.set(ControlMode.Velocity, 0);
                        collectorPiston.set(false);
                        break;
                    case COLLECT:
                        intakeMotor.set(ControlMode.Velocity, factory.getConstant(NAME, "collecting"));
                        break;
                    case EJECT:
                        intakeMotor.set(ControlMode.Velocity, factory.getConstant(NAME, "ejecting"));
                        break;
                }
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
        DOWN
    }

    public enum COLLECTOR_STATE {
        STOP,
        COLLECT,
        EJECT
    }
}

