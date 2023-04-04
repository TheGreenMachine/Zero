package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
    private GAME_ELEMENT currentGameElement = GAME_ELEMENT.NOTHING;

    private ROLLER_STATE desiredRollerState = ROLLER_STATE.STOP;
    private PIVOT_STATE desiredPivotState = PIVOT_STATE.STOW;
    private double rollerVelocity = 0;

    private double actualPivotPosition = 0;
    private boolean rollerOutputsChanged = false;
    private boolean pivotOutputsChanged = false;

    /**
     * Logging
     */
    private DoubleLogEntry rollerVelocityLogger;
    private DoubleLogEntry rollerCurrentDraw;
    private DoubleLogEntry pivotCurrentDraw;

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

        cubeIntakePower = factory.getConstant(NAME, "cubeIntakePower", 0.70);
        cubeOuttakePower = factory.getConstant(NAME, "cubeOuttakePower", 0.45);
        coneIntakePower = factory.getConstant(NAME, "coneIntakePower", -1.00);
        coneOuttakePower = factory.getConstant(NAME, "coneOuttakePower", 0.65);

        pivotStowPosition = factory.getConstant(NAME, "stowPosition", 1000);
        pivotScorePosition = factory.getConstant(NAME, "scorePosition", 1000);
        pivotShelfPosition = factory.getConstant(NAME, "shelfPosition", 1000);
        pivotFloorPosition = factory.getConstant(NAME, "floorPosition", 1000);

        intakeMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                true, factory.getConstant(NAME, "intakeStallAmps", 5), 0, 0),
            Constants.kCANTimeoutMs
        );

        intakeMotor.configOpenloopRamp(0.25, Constants.kCANTimeoutMs);

        allowablePivotError = factory.getPidSlotConfig(NAME, "slot1").allowableError;

        if (Constants.kLoggingRobot) {
            rollerVelocityLogger = new DoubleLogEntry(DataLogManager.getLog(), "Collector/Roller/actualRollerVelocity");
            desStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Collector/Pivot/desiredPivotPosition");
            actStatesLogger = new DoubleLogEntry(DataLogManager.getLog(), "Collector/Pivot/actualPivotPosition");
            rollerCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/currentDraw");
            pivotCurrentDraw = new DoubleLogEntry(DataLogManager.getLog(), "Elevator/currentDraw");
        }
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
     * Sets the desired roller state of the collector
     *
     * @param desiredRollerState STATE
     */
    public void setDesiredRollerState(ROLLER_STATE desiredRollerState) {
        this.desiredRollerState = desiredRollerState;
        rollerOutputsChanged = true;
    }

    /**
     * Sets the desired pivot state of the collector
     *
     * @param desiredPivotState STATE
     */
    public void setDesiredPivotState(PIVOT_STATE desiredPivotState) {
        this.desiredPivotState = desiredPivotState;
        pivotOutputsChanged = true;
    }

    /**
     * Sets the collector to outtake a game piece
     *
     * @param outtaking boolean
     */
    public void outtakeGamePiece(boolean outtaking) {
        if (outtaking) {
            setDesiredState(currentGameElement == GAME_ELEMENT.CONE ? ROLLER_STATE.OUTTAKE_CONE : ROLLER_STATE.OUTTAKE_CUBE, PIVOT_STATE.SCORE);
        } else {
            setDesiredState(ROLLER_STATE.STOP, PIVOT_STATE.STOW);
        }
    }

    /**
     * Returns the roller output
     */
    public double getRollerVelocity() {
        return rollerVelocity;
    }

    public double getActualPivotPosition() {
        return actualPivotPosition;
    }

    /**
     * Reads actual outputs from intake motor and solenoid
     *
     * @see Subsystem#readFromHardware()
     */
    @Override
    public void readFromHardware() {
        rollerVelocity = intakeMotor.getSelectedSensorVelocity(0);
        actualPivotPosition = pivotMotor.getSelectedSensorPosition(0);

        robotState.actualGameElement = currentGameElement;

        if (robotState.actualCollectorRollerState != desiredRollerState) {
            robotState.actualCollectorRollerState = desiredRollerState;
        }
        if (Math.abs(desiredPivotState.getPivotPosition() - actualPivotPosition) < allowablePivotError) {
            robotState.actualCollectorPivotState = desiredPivotState;
        }

        if (Constants.kLoggingRobot) {
            rollerVelocityLogger.append(rollerVelocity);

            ((DoubleLogEntry) desStatesLogger).append(desiredPivotState.getPivotPosition());
            ((DoubleLogEntry) actStatesLogger).append(actualPivotPosition);

            rollerCurrentDraw.append(intakeMotor.getOutputCurrent());
            pivotCurrentDraw.append(pivotMotor.getOutputCurrent());
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
                    if (currentGameElement == GAME_ELEMENT.CUBE) {
                        intakeMotor.set(ControlMode.PercentOutput, 0.05);
                    } else if (currentGameElement == GAME_ELEMENT.CONE) {
                        intakeMotor.set(ControlMode.PercentOutput, -0.05);
                    } else {
                        intakeMotor.set(ControlMode.PercentOutput, 0);
                    }
                }
                case INTAKE_CONE -> {
                    currentGameElement = GAME_ELEMENT.CONE;
                    intakeMotor.set(ControlMode.PercentOutput, coneIntakePower);
                }
                case INTAKE_CUBE -> {
                    currentGameElement = GAME_ELEMENT.CUBE;
                    intakeMotor.set(ControlMode.PercentOutput, cubeIntakePower);
                }
                case OUTTAKE_CONE -> {
                    currentGameElement = GAME_ELEMENT.NOTHING;
                    intakeMotor.set(ControlMode.PercentOutput, coneOuttakePower);
                }
                case OUTTAKE_CUBE -> {
                    currentGameElement = GAME_ELEMENT.NOTHING;
                    intakeMotor.set(ControlMode.PercentOutput, cubeOuttakePower);
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
//                    if (desiredRollerState == ROLLER_STATE.INTAKE_CONE) {
//                        pivotMotor.set(ControlMode.Position, (pivotShelfPosition+6));
//                    } else {
                        pivotMotor.set(ControlMode.Position, pivotShelfPosition);
//                    }
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
     * Returns the currently held game element
     *
     * @return game element
     */
    public GAME_ELEMENT getCurrentGameElement() {
        return currentGameElement;
    }

    /**
     * Returns the desired roller state
     *
     * @return desired roller state
     */
    public ROLLER_STATE getDesiredRollerState() {
        return desiredRollerState;
    }

    /**
     * Returns the desired pivot state
     *
     * @return desired pivot state
     */
    public PIVOT_STATE getDesiredPivotState() {
        return desiredPivotState;
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

    public enum GAME_ELEMENT {
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
