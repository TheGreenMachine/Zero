package com.team1816.lib.controlboard;

public class DancePadController extends Controller{
    /**
     * Instantiates a controller based on it's assigned (USB) port seen in the DriverStation utility
     *
     * @param port
     */
    public DancePadController(int port) {
        super(port);
        mJoystickButtonMap.put(Controller.Button.A, 5);
        mJoystickButtonMap.put(Controller.Button.B, 8);
        mJoystickButtonMap.put(Controller.Button.X, 7);
        mJoystickButtonMap.put(Controller.Button.Y, 6);
        mJoystickButtonMap.put(Controller.Button.SELECT, 9);
        mJoystickButtonMap.put(Controller.Button.START, 10);
        mJoystickButtonMap.put(Controller.Button.UP, 3);
        mJoystickButtonMap.put(Controller.Button.DOWN, 2);
        mJoystickButtonMap.put(Controller.Button.LEFT, 1);
        mJoystickButtonMap.put(Controller.Button.RIGHT, 4);
        mJoystickAxisMap.put(Controller.Axis.MIDDLE, 4);
    }
}
