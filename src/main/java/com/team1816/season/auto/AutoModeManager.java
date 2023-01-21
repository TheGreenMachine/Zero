package com.team1816.season.auto;

import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.auto.modes.DoNothingMode;
import com.team1816.season.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Singleton;

/**
 * An integrated and optimized manager for autonomous mode selection and configuration
 */
@Singleton
public class AutoModeManager {

    /**
     * Properties: Selection
     */
    private final SendableChooser<DesiredAuto> autoModeChooser;
    private final SendableChooser<Color> sideChooser;
    private DesiredAuto desiredAuto;
    private Color desiredColor;

    /**
     * Properties: Execution
     */
    private AutoMode autoMode;
    private static Thread autoModeThread;

    /**
     * Instantiates and AutoModeManager with a default option and selective computation
     */
    public AutoModeManager() {
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
    }

    /**
     * Updates the choosers in realtime
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
                System.out.println("Robot color: " + selectedColor);
            }
            autoMode = generateAutoMode(selectedAuto);
            autoModeThread = new Thread(autoMode::run);
        }
        desiredAuto = selectedAuto;
        desiredColor = selectedColor;

        return autoChanged;
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
     * @return AutoMode
     * @see AutoMode
     */
    public AutoMode getSelectedAuto() {
        return autoMode;
    }

    /**
     * Returns the selected color
     * @return Color
     * @see Color
     */
    public Color getSelectedColor() {
        return desiredColor;
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
        // Test : 2020
        DO_NOTHING,
        TUNE_DRIVETRAIN,
        LIVING_ROOM,
        DRIVE_STRAIGHT,
        // 2023
        AUTO_BALANCE
    }

    /**
     * Enum for bumper colors
     */
    public enum Color {
        RED,
        BLUE,
    }

    /**
     * Generates each AutoMode by demand
     * @param mode desiredMode
     * @return AutoMode
     * @see AutoMode
     */
    private AutoMode generateAutoMode(DesiredAuto mode) {
        switch (mode) {
            case DO_NOTHING:
                return new DoNothingMode();
            case TUNE_DRIVETRAIN:
                return new TuneDrivetrainMode();
            case LIVING_ROOM:
                return (new LivingRoomMode());
            case AUTO_BALANCE:
                return (new AutoBalanceMode());
            default:
                System.out.println("Defaulting to drive straight mode");
                return new DriveStraightMode();
        }
    }
}
