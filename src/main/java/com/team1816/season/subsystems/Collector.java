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
    public final double cubeIntakePower;
    public final double cubeOuttakePower;
    public final double coneIntakeVelocity;
    public final double coneOuttakeVelocity;

    /**
     * States
     */
    private STATE desiredState = STATE.STOP;
    private double rollerVelocity = 0;
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
        intakeSolenoid = factory.getSolenoid(NAME, "intakeSolenoid");
        intakeMotor = factory.getMotor(NAME, "intakeMotor");

        cubeIntakePower = factory.getConstant(NAME, "cubeIntakePower", 0.10); // TODO tune these
        cubeOuttakePower = factory.getConstant(NAME, "cubeOuttakePower", -0.25); // TODO tune these
        coneIntakeVelocity = factory.getConstant(NAME, "coneIntakeVelocity", -420); // TODO tune these
        coneOuttakeVelocity = factory.getConstant(NAME, "coneOuttakeVelocity", 840); // TODO tune these
    }

    public void setDesiredState(STATE desiredState) {
        if (this.desiredState != desiredState) {
            this.desiredState = desiredState;
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
        solenoidOutput = intakeSolenoid.get();

        // no checking performed
        if (robotState.actualCollectorState != desiredState) {
            robotState.actualCollectorState = desiredState;
        }
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
            switch (desiredState) {
                case STOP -> {
                    intakeSolenoid.set(false); // TODO check if down = true or false for each of these
                    intakeMotor.set(ControlMode.Velocity, 0);
                }
                case COL_CONE -> {
                    intakeSolenoid.set(true);
                    intakeMotor.set(ControlMode.Velocity, coneIntakeVelocity);
                }
                case COL_CUBE -> {
                    intakeSolenoid.set(false);
                    intakeMotor.set(ControlMode.PercentOutput, cubeIntakePower);
                }
                case FLUSH_CONE -> {
                    intakeSolenoid.set(false);
                    intakeMotor.set(ControlMode.Velocity, coneOuttakeVelocity);
                }
                case FLUSH_CUBE -> {
                    intakeSolenoid.set(false);
                    intakeMotor.set(ControlMode.PercentOutput, cubeOuttakePower);
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
    public enum STATE {
        STOP,
        COL_CUBE,
        COL_CONE,
        FLUSH_CUBE,
        FLUSH_CONE
    }
}
