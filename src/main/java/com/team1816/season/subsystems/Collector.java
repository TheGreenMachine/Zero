package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

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
    private final IGreenMotor intakeMotor; //TODO this one is REV

    private final IGreenMotor pivotMotor;

    /**
     * Properties
     */

    private final double collectorRevolutionsPerDegree;
    private final double zeroOffset;
    public final double cubeIntakePower;
    public final double cubeOuttakePower;

    public final double coneIntakePower;

    public final double coneOuttakePower;

    private static double pivotStowPosition;

    private static double pivotFloorPosition;

    private static double pivotConeShelfPosition;
    private static double pivotCubeShelfPosition;

    private static double pivotConeScorePosition;
    private static double pivotCubeScorePosition;
    private static double allowablePivotError;

    public static final double kCollectorLength = 0.35; // meters

    /**
     * States
     */
    public GAME_ELEMENT currentGameElement = GAME_ELEMENT.CONE;

    private ROLLER_STATE desiredRollerState = ROLLER_STATE.STOP;
    private PIVOT_STATE desiredPivotState = PIVOT_STATE.STOW;
    private double rollerVelocity = 0;

    private double desiredPivotPosition = 0;
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

        collectorRevolutionsPerDegree = factory.getConstant(NAME, "collectorRevolutionsPerDegree", 0);
        zeroOffset = factory.getConstant(NAME, "zeroOffset", 0);

        pivotStowPosition = (factory.getConstant(NAME, "stowAngle", 0) + zeroOffset) * collectorRevolutionsPerDegree;
        pivotConeScorePosition = (factory.getConstant(NAME, "scoreConeAngle", 0) + zeroOffset) * collectorRevolutionsPerDegree;
        pivotCubeScorePosition = (factory.getConstant(NAME, "scoreCubeAngle", 0) + zeroOffset) * collectorRevolutionsPerDegree;
        pivotConeShelfPosition = (factory.getConstant(NAME, "shelfConeAngle", 0) + zeroOffset) * collectorRevolutionsPerDegree;
        pivotCubeShelfPosition = (factory.getConstant(NAME, "shelfCubeAngle", 0) + zeroOffset) * collectorRevolutionsPerDegree;
        pivotFloorPosition = (factory.getConstant(NAME, "floorAngle", 0) + zeroOffset) * collectorRevolutionsPerDegree;

        intakeMotor.configOpenLoopRampRate(0.25, Constants.kCANTimeoutMs);

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

    public void setCurrentGameElement(GAME_ELEMENT currentGameElement) {
        this.currentGameElement = currentGameElement;
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

    public double getDesiredPivotPosition() {
        return desiredPivotPosition;
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
        rollerVelocity = intakeMotor.getSensorVelocity(0);
        actualPivotPosition = pivotMotor.getSensorPosition(0);

        robotState.gameElementChanged = robotState.actualGameElement != currentGameElement;
        robotState.actualGameElement = currentGameElement;
        robotState.actualCollectorAngle = actualPivotPosition / collectorRevolutionsPerDegree - zeroOffset;


        if (robotState.actualCollectorRollerState != desiredRollerState) {
            robotState.actualCollectorRollerState = desiredRollerState;
        }
        if (Math.abs(desiredPivotPosition - actualPivotPosition) < allowablePivotError && !pivotOutputsChanged) {
            robotState.actualCollectorPivotState = desiredPivotState;
        }

        if (Constants.kLoggingRobot) {
            rollerVelocityLogger.append(rollerVelocity);

            ((DoubleLogEntry) desStatesLogger).append(desiredPivotPosition);
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
                        intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, 0.05);
                    } else if (currentGameElement == GAME_ELEMENT.CONE) {
                        intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, -0.1);
                    } else {
                        intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, 0);
                    }
                }
                case INTAKE_CONE -> {
                    currentGameElement = GAME_ELEMENT.CONE;
                    intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, coneIntakePower);
                }
                case INTAKE_CUBE -> {
                    currentGameElement = GAME_ELEMENT.CUBE;
                    intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, cubeIntakePower);
                }
                case OUTTAKE_CONE -> {
                    currentGameElement = GAME_ELEMENT.NOTHING;
                    intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, coneOuttakePower);
                }
                case OUTTAKE_CUBE -> {
                    currentGameElement = GAME_ELEMENT.NOTHING;
                    intakeMotor.set(GreenControlMode.PERCENT_OUTPUT, cubeOuttakePower);
                }
            }
        }
        if (pivotOutputsChanged) {
            pivotOutputsChanged = false;
            double pos = 0;
            switch (desiredPivotState) {
                case STOW -> {
                    pos = pivotStowPosition;
                }
                case FLOOR -> {
                    pos = pivotFloorPosition;
                }
                case SCORE -> {
                    if (currentGameElement == GAME_ELEMENT.CUBE) {
                        pos = pivotCubeScorePosition;
                    } else {
                        pos = pivotConeScorePosition;
                    }
                }
                case SHELF -> {
                    if (currentGameElement == GAME_ELEMENT.CONE) {
                        pos = pivotConeShelfPosition;
                    } else {
                        pos = pivotCubeShelfPosition;
                    }
                }
            }
            desiredPivotPosition = pos;

            pivotMotor.set(GreenControlMode.POSITION_CONTROL, pos);
        }
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {
        pivotMotor.setSensorPosition(0, 0, Constants.kCANTimeoutMs);
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
        STOW,
        FLOOR,
        SHELF,
        SCORE;
    }
}
