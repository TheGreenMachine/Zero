package com.team1816.season.auto;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.PathFinder;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.auto.modes.DoNothingMode;
import com.team1816.lib.auto.modes.DriveStraightMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.modes.*;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.List;

/**
 * An integrated and optimized manager for autonomous mode selection and configuration
 */
@Singleton
public class AutoModeManager {

    /**
     * Properties: Selection
     */
    public static com.team1816.season.states.RobotState robotState;
    private final SendableChooser<DesiredAuto> autoModeChooser;
    private final SendableChooser<Color> sideChooser;
    private DesiredAuto desiredAuto;
    private Color teamColor;

    /**
     * Properties: Execution
     */
    private AutoMode autoMode;
    private static Thread autoModeThread;

    /**
     * Instantiates and AutoModeManager with a default option and selective computation
     *
     * @param rs RobotState
     */
    @Inject
    public AutoModeManager(com.team1816.season.states.RobotState rs) {
        robotState = rs;
        autoModeChooser = new SendableChooser<>(); // Shuffleboard dropdown menu to choose desired auto mode
        sideChooser = new SendableChooser<>(); // Shuffleboard dropdown menu to choose desired side / bumper color

        SmartDashboard.putData("Auto mode", autoModeChooser); // appends chooser to shuffleboard

        for (DesiredAuto desiredAuto : DesiredAuto.values()) {
            autoModeChooser.addOption(desiredAuto.name(), desiredAuto);
        }
        autoModeChooser.setDefaultOption(
            DesiredAuto.DRIVE_STRAIGHT.name(),
            DesiredAuto.DRIVE_STRAIGHT
        );

        SmartDashboard.putData("Robot color", sideChooser); // appends chooser to shuffleboard

        sideChooser.setDefaultOption(Color.BLUE.name(), Color.BLUE); // initialize options
        sideChooser.addOption(Color.RED.name(), Color.RED); // initialize options

        reset();
    }

    /**
     * Resets properties to default and resets the thread
     */
    public void reset() {
        autoMode = new DriveStraightMode();
        autoModeThread = new Thread(autoMode::run);
        desiredAuto = DesiredAuto.DRIVE_STRAIGHT;
        teamColor = Color.RED;
        robotState.allianceColor = teamColor;
    }

    /**
     * Updates the choosers in realtime
     *
     * @return true if updated
     */
    public boolean update() {
        DesiredAuto selectedAuto = autoModeChooser.getSelected();
        Color selectedColor = Color.BLUE;
        if (RobotBase.isSimulation()) {
            selectedColor = sideChooser.getSelected();
        } else if (RobotBase.isReal()) {
            var dsAlliance = DriverStation.getAlliance();
            selectedColor = (dsAlliance == DriverStation.Alliance.Red) ? Color.RED : Color.BLUE;
        }
        boolean autoChanged = desiredAuto != selectedAuto;
        boolean colorChanged = teamColor != selectedColor;
        // if auto has been changed, update selected auto mode + thread
        if (autoChanged || colorChanged) {
            if (autoChanged) {
                GreenLogger.log(
                    "Auto changed from: " + desiredAuto + ", to: " + selectedAuto.name()
                );
            }
            if (colorChanged) {
                GreenLogger.log("Robot color changed from: " + teamColor + ", to: " + selectedColor);
            }
            autoMode = generateAutoMode(selectedAuto, selectedColor);
            autoModeThread = new Thread(autoMode::run);
        }
        desiredAuto = selectedAuto;
        teamColor = selectedColor;
        robotState.allianceColor = teamColor;

        if (robotState.allianceColor == Color.BLUE) {
            robotState.pathFinder = new PathFinder(List.of(Constants.blueChargeStation));
        } else {
            robotState.pathFinder = new PathFinder(List.of(Constants.redChargeStation));
        }

        return autoChanged || colorChanged;
    }

    /**
     * Outputs values to SmartDashboard
     */
    public void outputToSmartDashboard() {
        if (desiredAuto != null) {
            SmartDashboard.putString("AutoModeSelected", desiredAuto.name());
        }
        if (teamColor != null) {
            SmartDashboard.putString("RobotColorSelected", teamColor.name());
        }
    }

    /**
     * Returns the selected autonomous mode
     *
     * @return AutoMode
     * @see AutoMode
     */
    public AutoMode getSelectedAuto() {
        return autoMode;
    }

    /**
     * Returns the selected color
     *
     * @return Color
     * @see Color
     */
    public Color getSelectedColor() {
        return sideChooser.getSelected();
    }

    /**
     * Executes the auto mode and respective thread
     */
    public void startAuto() {
        autoModeThread.start();
    }

    /**
     * Stops the auto mode
     */
    public void stopAuto() {
        if (autoMode != null) {
            autoMode.stop();
            autoModeThread = new Thread(autoMode::run);
        }
    }

    /**
     * Enum for AutoModes
     */
    enum DesiredAuto {
        // Test : 2020 Legacy
        DO_NOTHING,
        TUNE_DRIVETRAIN,
        LIVING_ROOM,
        DRIVE_STRAIGHT,

        // New Auto Modes : 20??
    }

    /**
     * Generates each AutoMode by demand
     *
     * @param mode desiredMode
     * @return AutoMode
     * @see AutoMode
     */
    private AutoMode generateAutoMode(DesiredAuto mode, Color color) {
        switch (mode) {
            case DO_NOTHING:
                return new DoNothingMode();
//            case TUNE_DRIVETRAIN: // commented for competition purposes
//                return new TuneDrivetrainMode();
//            case LIVING_ROOM:
//                return (new LivingRoomMode(color));
            default:
                GreenLogger.log("Defaulting to drive straight mode");
                return new DriveStraightMode();
        }
    }
}
