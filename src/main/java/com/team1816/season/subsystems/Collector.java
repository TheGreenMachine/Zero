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

    //piston is for the switch from cone collecting to carrying + collecting cubes
    private final ISolenoid collectorPiston;
    private final IGreenMotor intakeMotor;
    private double intakeVel;

    // State
    private PIVOT_STATE desiredPivotState = PIVOT_STATE.DOWN;
    private COLLECTOR_STATE desiredCollectorState = COLLECTOR_STATE.STOP;
    //this is for negating the direction for cone/cube
    private boolean isCube;
    private boolean outputsChanged = false;

    // Consts
    private static final String NAME = "collector";
    public final double cubeCollectPow;
    public final double cubeEjectPow;
    public final double coneCollectVel;
    public final double coneEjectVel;

    @Inject
    public Collector (Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        collectorPiston = factory.getSolenoid(NAME, "collectorSolenoid");
        intakeMotor = factory.getMotor(NAME, "intakeMotor");

        cubeCollectPow = factory.getConstant(NAME, "cubeCollectPow", 0.05); // TODO tune these
        cubeEjectPow = factory.getConstant(NAME, "cubeEjectPow", -0.25); // TODO tune these
        coneCollectVel = factory.getConstant(NAME, "coneCollectVel", -420); // TODO tune these
        coneEjectVel = factory.getConstant(NAME, "coneEjectVel", 840); // TODO tune these
    }

    public void setDesiredState(boolean isCube, PIVOT_STATE pivotState, COLLECTOR_STATE collectorState){
        this.isCube = isCube;

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
        intakeVel = intakeMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
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
                    intakeMotor.set(
                        isCube ? ControlMode.PercentOutput : ControlMode.Velocity,
                        isCube ? cubeCollectPow : coneCollectVel
                    );
                    break;
                case EJECT:
                    intakeMotor.set(
                            isCube ? ControlMode.PercentOutput : ControlMode.Velocity,
                            isCube ? cubeEjectPow : coneEjectVel
                    );
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

