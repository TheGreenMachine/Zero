package com.team1816.lib.controlboard;

/**
 * A mapping of outputs for a WasdController
 * @see Controller
 */
public class ButtonboardController extends Controller {

    /**
     * Instantiates a controller based on it's assigned (USB) port seen in the DriverStation utility
     *
     * @param port
     */
    public ButtonboardController(int port) {
        super(port);
        mJoystickButtonMap.put(Controller.Button.UP_LEFT, 0);
        mJoystickButtonMap.put(Controller.Button.UP, 1);
        mJoystickButtonMap.put(Controller.Button.UP_RIGHT, 2);
        mJoystickButtonMap.put(Controller.Button.LEFT, 3);
        mJoystickButtonMap.put(Controller.Button.CENTER, 4);
        mJoystickButtonMap.put(Controller.Button.RIGHT, 5);
        mJoystickButtonMap.put(Controller.Button.DOWN_LEFT, 6);
        mJoystickButtonMap.put(Controller.Button.DOWN, 7);
        mJoystickButtonMap.put(Controller.Button.DOWN_RIGHT, 8);
    }
}
