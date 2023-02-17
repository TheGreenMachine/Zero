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

    /**
     * Name
     */
    private static final String NAME = "collector";

    /**
     * Components
     */
    private final ISolenoid intakeSolenoid;
    private final IGreenMotor intakeMotor;

    /**
     * Properties
     */
    public final double cubeIntakeVelocity;
    public final double cubeOuttakeVelocity;
    public final double coneIntakeVelocity;
    public final double coneOuttakeVelocity;

    /**
     * States
     */
    private ControlMode desiredControlMode = ControlMode.PercentOutput;
    private PIVOT_STATE desiredPivotState = PIVOT_STATE.DOWN;
    private ROLLER_STATE desiredRollerState = ROLLER_STATE.STOP;
    private double rollerVelocity = 0;
    private boolean outputsChanged = false;

    /**
     * Base constructor needed to instantiate a collector
     *
     * @param inf Infrastructure
     * @param rs  RobotState
     */
    @Inject
    public Collector(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        intakeSolenoid = factory.getSolenoid(NAME, "intakeSolenoid");
        intakeMotor = factory.getMotor(NAME, "intakeMotor");

        cubeIntakeVelocity = factory.getConstant(NAME, "cubeIntakePower", 0.10); // TODO tune these
        cubeOuttakeVelocity = factory.getConstant(NAME, "cubeOuttakePower", -0.25); // TODO tune these
        coneIntakeVelocity = factory.getConstant(NAME, "coneIntakeVelocity", -420); // TODO tune these
        coneOuttakeVelocity = factory.getConstant(NAME, "coneOuttakeVelocity", 840); // TODO tune these
    }

    /**
     * Sets the desired state of the collector
     *
     * @param controlMode desired control mode
     * @param pivotState  desired collector pivot state
     * @param rollerState desired collector roller state
     */
    public void setDesiredState(ControlMode controlMode, PIVOT_STATE pivotState, ROLLER_STATE rollerState) {
        if (desiredControlMode != controlMode) {
            desiredControlMode = controlMode;
            outputsChanged = true;
        }
        if (desiredPivotState != pivotState) {
            desiredPivotState = pivotState;
            outputsChanged = true;
        }
        if (desiredRollerState != rollerState) {
            desiredRollerState = rollerState;
            outputsChanged = true;
        }
    }

    /**
     * Reads roller velocity
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        rollerVelocity = intakeMotor.getSelectedSensorVelocity(0);
    }

    /**
     * Writes outputs to collector motor and solenoid
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredPivotState) {
                case UP:
                    intakeSolenoid.set(false);
                    break;
                case DOWN:
                    intakeSolenoid.set(true);
                    break;
            }
            switch (desiredRollerState) {
                case STOP:
                    intakeMotor.set(ControlMode.Velocity, 0);
                    intakeSolenoid.set(false);
                    break;
                case INTAKE:
                    intakeMotor.set(
                        desiredControlMode,
                        (desiredControlMode == ControlMode.PercentOutput) ? cubeIntakeVelocity : coneIntakeVelocity
                    );
                    break;
                case OUTTAKE:
                    intakeMotor.set(
                        desiredControlMode,
                        (desiredControlMode == ControlMode.Velocity) ? cubeOuttakeVelocity : coneOuttakeVelocity
                    );
                    break;
            }
        }
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {

    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {

    }

    /**
     * Tests the collector subsystem, returns true if tests passed
     *
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        return false;
    }

    /**
     * Base enums for collector
     */
    public enum PIVOT_STATE {
        UP,
        DOWN
    }

    public enum ROLLER_STATE {
        STOP,
        INTAKE,
        OUTTAKE
    }
}
