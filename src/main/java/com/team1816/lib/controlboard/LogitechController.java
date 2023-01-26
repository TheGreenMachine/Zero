package com.team1816.lib.controlboard;

/**
 * A mapping of outputs for a LogitechController
 *
 * @see Controller
 */
public class LogitechController extends Controller {

    public LogitechController(int port) {
        super(port);
        mJoystickButtonMap.put(Controller.Button.A, 1);
        mJoystickButtonMap.put(Controller.Button.B, 2);
        mJoystickButtonMap.put(Controller.Button.X, 3);
        mJoystickButtonMap.put(Controller.Button.Y, 4);
        mJoystickButtonMap.put(Controller.Button.LEFT_BUMPER, 5);
        mJoystickButtonMap.put(Controller.Button.RIGHT_BUMPER, 6);
        mJoystickButtonMap.put(Controller.Button.BACK, 9);
        mJoystickButtonMap.put(Controller.Button.START, 10);
        mJoystickButtonMap.put(Controller.Button.L_JOYSTICK, 11);
        mJoystickButtonMap.put(Controller.Button.R_JOYSTICK, 12);
        mJoystickAxisMap.put(Axis.LEFT_X, 0);
        mJoystickAxisMap.put(Axis.LEFT_Y, 1);
        mJoystickAxisMap.put(Axis.RIGHT_X, 2);
        mJoystickAxisMap.put(Axis.RIGHT_Y, 3);
    }
}
