package com.team1816.lib.controlboard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import java.util.EnumMap;

/**
 * Base abstract class that interfaces with a controller
 * @see LogitechController
 * @see WasdController
 * @see XboxController
 */
public abstract class Controller {

    /**
     * Base factory for controllers
     */
    public interface Factory {
        Controller getControllerInstance(int port);
    }

    /** State */
    protected final Joystick mController;
    public static final double kAxisThreshold = 0.04;
    public static final double kJoystickBooleanThreshold = 0.80;
    protected final EnumMap<Button, Integer> mJoystickButtonMap = new EnumMap<>(
        Button.class
    );
    protected final EnumMap<Axis, Integer> mJoystickAxisMap = new EnumMap<>(Axis.class);

    /**
     * Enum for axes on a controller
     */
    public enum Axis {
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER,
    }

    /**
     * Enum for buttons on a controller
     */
    public enum Button {
        // Standard Controls
        A,
        B,
        X,
        Y,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        BACK,
        START,
        L_JOYSTICK,
        R_JOYSTICK,

        // Button Board
        CENTER,
        UP,
        DOWN,
        LEFT,
        RIGHT,
        UP_LEFT,
        UP_RIGHT,
        DOWN_LEFT,
        DOWN_RIGHT

    }

    /**
     * Instantiates a controller based on it's assigned (USB) port seen in the DriverStation utility
     * @param port
     */
    public Controller(int port) {
        mController = new Joystick(port);
    }

    /**
     * Toggles rumble on a generic controller, can be fine tuned further
     * @param on
     */
    public void setRumble(boolean on) {
        mController.setRumble(GenericHID.RumbleType.kRightRumble, on ? 1 : 0);
    }

    /**
     * Returns dpad outputs in a standardized integer format
     * @return dpad
     */
    public int getDPad() {
        return mController.getPOV();
    }

    /**
     * Returns button outputs
     * @param button
     * @return boolean buttonOutput (is the button pressed)
     */
    public boolean getButton(Button button) {
        if (!mJoystickButtonMap.containsKey(button)) return false;
        return mController.getRawButton(mJoystickButtonMap.get(button));
    }

    /**
     * Returns axis outputs on a boolean basis based on the {@link Controller#kAxisThreshold}
     * @param axis
     * @return
     */
    public boolean getTrigger(Axis axis) {
        if (!mJoystickAxisMap.containsKey(axis)) return false;
        return (mController.getRawAxis(mJoystickAxisMap.get(axis)) > kAxisThreshold);
    }

    /**
     * Returns joystick / axis output as a double
     * @param axis
     * @return
     */
    public double getJoystick(Axis axis) {
        if (!mJoystickAxisMap.containsKey(axis)) return 0;
        return mController.getRawAxis(mJoystickAxisMap.get(axis));
    }

    /**
     * Returns joystick / axis output with an additional inverted parameter
     * @param axis
     * @param inverted
     * @return
     */
    public double getJoystick(Axis axis, boolean inverted) {
        return (
            (inverted ? (-1) : (1)) * mController.getRawAxis(mJoystickAxisMap.get(axis))
        );
    }
}
