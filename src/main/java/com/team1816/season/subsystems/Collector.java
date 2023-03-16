package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public final double coneIntakePower;

    public final double coneOuttakePower;

    private static double pivotStowPosition;

    private static double pivotFloorPosition;

    private static double pivotShelfPosition;

    private static double pivotScorePosition;

    private static double allowablePivotError;


    /**
     * States
     */
    private ROLLER_STATE desiredRollerState = ROLLER_STATE.STOP;

    private PIVOT_STATE desiredPivotState = PIVOT_STATE.STOW;
    private GAME_ELEMENT currentlyHeldObject = GAME_ELEMENT.NOTHING;    //remember, game_element and state enums
    private double rollerVelocity = 0;

    private double actualPivotPosition = 0;
    private boolean solenoidOutput = false;
    private boolean rollerOutputsChanged = false;

    private boolean pivotOutputsChanged = false;

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
        pivotMotor = factory.getMotor(NAME, "pivotMotor");

        cubeIntakePower = factory.getConstant(NAME, "cubeIntakePower", -0.10); // TODO tune these
        cubeOuttakePower = factory.getConstant(NAME, "cubeOuttakePower", 0.25); // TODO tune these
        coneIntakePower = factory.getConstant(NAME, "coneIntakePower", -0.25); // TODO tune these
        coneOuttakePower = factory.getConstant(NAME, "coneOuttakePower", 0.25); // TODO tune these

        pivotStowPosition = factory.getConstant(NAME, "stowPosition", 1000);
        pivotScorePosition = factory.getConstant(NAME, "scorePosition", 1000);
        pivotShelfPosition = factory.getConstant(NAME, "shelfPosition", 1000);
        pivotFloorPosition = factory.getConstant(NAME, "floorPosition", 1000);

        allowablePivotError = factory.getPidSlotConfig(NAME, "slot1").allowableError;
    }

    /**
     * Sets the desired state of the collector
     *
     * @param desiredRollerState STATE
     */
    public void setDesiredState(ROLLER_STATE desiredRollerState, PIVOT_STATE desiredPivotState) {
        this.desiredRollerState = desiredRollerState;
        this.desiredPivotState = desiredPivotState;
        rollerOutputsChanged = true;
        pivotOutputsChanged = true;
    }

    /**
     * Sets the collector to outtake a game piece
     *
     * @param outtaking boolean
     */
    public void outtakeGamePiece(boolean outtaking) {
        if (outtaking) {
            setDesiredState(currentlyHeldObject == GAME_ELEMENT.CONE ? ROLLER_STATE.OUTTAKE_CONE : ROLLER_STATE.OUTTAKE_CUBE, PIVOT_STATE.SCORE);
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

    public double getActualPivotPosition() { return actualPivotPosition; }

    /**
     * Reads actual outputs from intake motor and solenoid
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        rollerVelocity = intakeMotor.getSelectedSensorVelocity(0);
        actualPivotPosition = pivotMotor.getSelectedSensorPosition(0);

        if (robotState.actualCollectorRollerState != desiredRollerState) {
            robotState.actualCollectorRollerState = desiredRollerState;
        }
        if (Math.abs(desiredPivotState.getPivotPosition() - actualPivotPosition) < allowablePivotError) {
            robotState.actualCollectorPivotState = desiredPivotState;
        }
    }

    /**
     * Writes outputs to intake motor and solenoid
     *
     * @see Subsystem#writeToHardware()
     */
    @Override
    public void writeToHardware() {
        if (rollerOutputsChanged) {
            rollerOutputsChanged = false;
            switch (desiredRollerState) {
                case STOP -> {
                    intakeMotor.set(ControlMode.Velocity, 0);
                }
                case INTAKE_CONE -> {
                    currentlyHeldObject = GAME_ELEMENT.CONE;
                    intakeMotor.set(ControlMode.Velocity, coneIntakePower);
                }
                case INTAKE_CUBE -> {
                    currentlyHeldObject = GAME_ELEMENT.CUBE;
                    intakeMotor.set(ControlMode.Velocity, cubeIntakePower);
                }
                case OUTTAKE_CONE -> {
                    intakeMotor.set(ControlMode.Velocity, coneOuttakePower);
//                    currentlyHeldObject = GAME_ELEMENT.NOTHING;
                }
                case OUTTAKE_CUBE -> {
                    intakeMotor.set(ControlMode.Velocity, cubeOuttakePower);
//                    currentlyHeldObject = GAME_ELEMENT.NOTHING;
                }
            }
        }
        if (pivotOutputsChanged) {
            pivotOutputsChanged = false;
            switch (desiredPivotState) {
                case STOW -> {
                    pivotMotor.set(ControlMode.Position, pivotStowPosition);
                }
                case FLOOR -> {
                    pivotMotor.set(ControlMode.Position, pivotFloorPosition);
                }
                case SCORE -> {
                    pivotMotor.set(ControlMode.Position, pivotScorePosition);
                }
                case SHELF -> {
                    pivotMotor.set(ControlMode.Position, pivotShelfPosition);
                }
            }
        }
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {
        pivotMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
    }

    /**
     * Sets the collector pivot to break
     */
    public void setBraking(boolean braking) {
        pivotMotor.setNeutralMode(braking ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {

    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Collector/Collector Pivot Position", actualPivotPosition);
        SmartDashboard.putNumber("Collector/Collector Roller Velocity", rollerVelocity);
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
        STOW(pivotStowPosition),
        FLOOR(pivotFloorPosition),
        SHELF(pivotShelfPosition),
        SCORE(pivotScorePosition);

        private final double pivot;

        PIVOT_STATE(double pivot) {
            this.pivot = pivot;
        }

        public double getPivotPosition() {
            return pivot;
        }
    }
}
