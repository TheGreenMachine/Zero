package com.team1816.lib.controlboard;

/**
 * A mapping of outputs for a WasdController
 *
 * @see Controller
 */
public class WasdController extends Controller {

    public WasdController(int port) {
        super(port);
        mJoystickButtonMap.put(Controller.Button.A, 2);
        mJoystickButtonMap.put(Controller.Button.B, 3);
        mJoystickButtonMap.put(Controller.Button.X, 1);
        mJoystickButtonMap.put(Controller.Button.Y, 4);
        mJoystickButtonMap.put(Controller.Button.LEFT_BUMPER, 5);
        mJoystickButtonMap.put(Controller.Button.RIGHT_BUMPER, 6);
        mJoystickButtonMap.put(Controller.Button.BACK, 9);
        mJoystickButtonMap.put(Controller.Button.START, 10);
        mJoystickButtonMap.put(Controller.Button.L_JOYSTICK, 11);
        mJoystickButtonMap.put(Controller.Button.R_JOYSTICK, 12);
        mJoystickAxisMap.put(Axis.RIGHT_X, 0);
        mJoystickAxisMap.put(Axis.LEFT_X, 1);
        mJoystickAxisMap.put(Axis.LEFT_Y, 2);
    }
}
