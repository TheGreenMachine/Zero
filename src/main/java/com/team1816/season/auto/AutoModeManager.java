package com.team1816.season.auto;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.modes.*;
import com.team1816.season.auto.modes.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.inject.Inject;
import javax.inject.Singleton;

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
    private Color desiredColor;

    /**
     * Properties: Execution
     */
    private AutoMode autoMode;
    private static Thread autoModeThread;
    private RobotState robotState1;

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
        desiredColor = Color.RED;
        robotState.allianceColor = desiredColor;
    }

    /**
     * Updates the choosers in realtime
     *
     * @return true if updated
     */
    public boolean update() {
        DesiredAuto selectedAuto = autoModeChooser.getSelected();
        Color selectedColor = sideChooser.getSelected();
        boolean autoChanged = desiredAuto != selectedAuto;
        boolean colorChanged = desiredColor != selectedColor;
        // if auto has been changed, update selected auto mode + thread
        if (autoChanged || colorChanged) {
            if (autoChanged) {
                System.out.println(
                    "Auto changed from: " + desiredAuto + ", to: " + selectedAuto.name()
                );
            }
            if (colorChanged) {
                System.out.println("Robot color changed from: " + desiredColor + ", to: " + selectedColor);
            }
            autoMode = generateAutoMode(selectedAuto, selectedColor);
            autoModeThread = new Thread(autoMode::run);
        }
        desiredAuto = selectedAuto;
        desiredColor = selectedColor;
        robotState.allianceColor = desiredColor;

        return autoChanged || colorChanged;
    }

    /**
     * Outputs values to SmartDashboard
     */
    public void outputToSmartDashboard() {
        if (desiredAuto != null) {
            SmartDashboard.putString("AutoModeSelected", desiredAuto.name());
        }
        if (desiredColor != null) {
            SmartDashboard.putString("RobotColorSelected", desiredColor.name());
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

        // 2023
        PLACE_CONE,
        EXIT_COMMUNITY,
        EXIT_BALANCE_FEEDER,
        EXIT_BALANCE_MIDDLE,
        EXIT_BALANCE_WALL,
        PLACE_CONE_AUTO_BALANCE_FEEDER,
        PLACE_CONE_AUTO_BALANCE_MIDDLE,
        PLACE_CONE_AUTO_BALANCE_WALL,
        DOUBLE_CONE_FEEDER,
        DOUBLE_CONE_WALL
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
//            case TUNE_DRIVETRAIN: // comented for competition purposes
//                return new TuneDrivetrainMode();
//            case LIVING_ROOM:
//                return (new LivingRoomMode(color));
            case EXIT_COMMUNITY:
                return (new NodeToExitCommunityMode(color));
            case PLACE_CONE:
                return (new PlaceConeMode());
            case EXIT_BALANCE_FEEDER:
                return (new ExitCommunityBalanceFeederMode(color));
            case EXIT_BALANCE_MIDDLE:
                return (new ExitCommunityBalanceMiddleMode(color));
            case EXIT_BALANCE_WALL:
                return (new ExitCommunityBalanceWallMode(color));
            case PLACE_CONE_AUTO_BALANCE_FEEDER:
                return (new PlaceConeAutoBalanceFeederMode(color));
            case PLACE_CONE_AUTO_BALANCE_MIDDLE:
                return (new PlaceConeAutoBalanceMiddleMode(color));
            case PLACE_CONE_AUTO_BALANCE_WALL:
                return (new PlaceConeAutoBalanceWallMode(color));
            case DOUBLE_CONE_FEEDER:
                return (new DoublePlaceConeFeederMode(color));
            case DOUBLE_CONE_WALL:
                return (new DoublePlaceConeWallMode(color));
            default:
                System.out.println("Defaulting to drive straight mode");
                return new DriveStraightMode();
        }
    }
}
