package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * Represents a Collector with a solenoid and rollers powered by a motor to collect both a cone and a cube
 */
@Singleton
public class Collector extends Subsystem {

    /**
     * Name
     */
    private static final String NAME = "collector";

    /**
     * Components
     */
    private final IGreenMotor intakeMotor;

    private final IGreenMotor pivotMotor;

    /**
     * Properties
     */
    public final double cubeIntakePower;
    public final double cubeOuttakePower;
    public final double coneIntakeVelocity;
    public final double coneOuttakeVelocity;

    public final double collectPosition;

    public final double stowPosition;

    /**
     * States
     */
    private ROLLER_STATE desiredRollerState = ROLLER_STATE.STOP;

    private PIVOT_STATE  = PIVOT_STATE.STOW;
    private GAME_ELEMENT currentlyHeldObject = GAME_ELEMENT.NOTHING;    //remember, game_element and state enums
    private double rollerVelocity = 0;

    private double pivotPosition = 0;
    private boolean solenoidOutput = false;
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
        intakeMotor = factory.getMotor(NAME, "intakeMotor");
        pivotMotor = factory.getMotor(NAME, "hingeMotor");

        cubeIntakePower = factory.getConstant(NAME, "cubeIntakePower", 0.10); // TODO tune these
        cubeOuttakePower = factory.getConstant(NAME, "cubeOuttakePower", -0.25); // TODO tune these
        coneIntakeVelocity = factory.getConstant(NAME, "coneIntakeVelocity", -420); // TODO tune these
        coneOuttakeVelocity = factory.getConstant(NAME, "coneOuttakeVelocity", 840); // TODO tune these

        collectPosition = factory.getConstant(NAME, "collectPosition", 1000);
        stowPosition = factory.getConstant(NAME, "stowPosition", 1000);
    }

    /**
     * Sets the desired state of the collector
     *
     * @param desiredROLLERState STATE
     */
    public void setDesiredState(ROLLER_STATE desiredROLLERState, PIVOT_STATE desiredPIVOTState) {
        this.desiredRollerState = desiredROLLERState;
        this. = desiredPIVOTState;
        outputsChanged = true;
    }

    /**
     * Sets the collector to outtake a game piece
     *
     * @param outtaking boolean
     */
    public void outtakeGamePiece(boolean outtaking) {
        if (outtaking) {
            setDesiredState(currentlyHeldObject == GAME_ELEMENT.CONE ? ROLLER_STATE.OUTTAKE_CONE : ROLLER_STATE.OUTTAKE_CUBE, PIVOT_STATE.ENGAGED);
        } else {
            setDesiredState(ROLLER_STATE.STOP, PIVOT_STATE.STOW);
        }
    }

    /**
     * Returns the actual output of the solenoid
     *
     * @return 1 if firing, 0 otherwise
     */
    public double getSolenoidOutput() {
        return solenoidOutput ? 1 : 0;
    }

    /**
     * Returns the roller output
     */
    public double getRollerVelocity() {
        return rollerVelocity;
    }

    public double getPivotPosition() { return pivotPosition; }

    /**
     * Reads actual outputs from intake motor and solenoid
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        rollerVelocity = intakeMotor.getSelectedSensorVelocity(0);
        pivotPosition = pivotMotor.getSelectedSensorPosition(0);
        if (robotState.actualCollectorRollerState != desiredRollerState && pivotPosition - pivotPosition) {
            robotState.actualCollectorRollerState = desiredRollerState;
        }
    }

    /**
     * Writes outputs to intake motor and solenoid
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            pivotMotor.set(ControlMode.Velocity, 0);
            switch (desiredRollerState) {
                case STOP -> {
                    intakeMotor.set(ControlMode.Velocity, 0);
                    pivotMotor.set(ControlMode.Position, )
                }
                case INTAKE_CONE -> {
                    currentlyHeldObject = GAME_ELEMENT.CONE;
                    intakeMotor.set(ControlMode.Velocity, coneIntakeVelocity);
                }
                case INTAKE_CUBE -> {
                    currentlyHeldObject = GAME_ELEMENT.CUBE;
                    intakeMotor.set(ControlMode.PercentOutput, cubeIntakePower);
                }
                case OUTTAKE_CONE -> {
                    intakeMotor.set(ControlMode.Velocity, coneOuttakeVelocity);
//                    currentlyHeldObject = GAME_ELEMENT.NOTHING;
                }
                case OUTTAKE_CUBE -> {
                    intakeMotor.set(ControlMode.PercentOutput, cubeOuttakePower);
//                    currentlyHeldObject = GAME_ELEMENT.NOTHING;
                }
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
     * Base enum for collector
     */

    // should do exactly what it used to - just condensed the states into 5 instead of 3 separate for each enum
    public enum ROLLER_STATE {
        STOP,
        INTAKE_CUBE,
        INTAKE_CONE,
        OUTTAKE_CUBE,
        OUTTAKE_CONE
    }

    private enum GAME_ELEMENT {
        NOTHING,
        CUBE,
        CONE
    }

    public enum PIVOT_STATE {
        STOW,
        FLOOR,
        SHELF,
        SCORE
    }
}
