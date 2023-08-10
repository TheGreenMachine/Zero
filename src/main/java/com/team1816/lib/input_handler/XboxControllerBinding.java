package com.team1816.lib.input_handler;

import com.team1816.lib.controlboard.Controller;

public class XboxControllerBinding extends ControllerBinding {
    public XboxControllerBinding() {
        buttonMap.put(Button.A, 1);
        buttonMap.put(Button.B, 2);
        buttonMap.put(Button.X, 3);
        buttonMap.put(Button.Y, 4);
        buttonMap.put(Button.LEFT_BUMPER, 5);
        buttonMap.put(Button.RIGHT_BUMPER, 6);
        buttonMap.put(Button.BACK, 7);
        buttonMap.put(Button.START, 8);
        buttonMap.put(Button.L_JOYSTICK, 9);
        buttonMap.put(Button.R_JOYSTICK, 10);
        axisMap.put(Axis.LEFT_HORIZONTAL, 0);
        axisMap.put(Axis.LEFT_VERTICAL, 1);
        axisMap.put(Axis.LEFT_TRIGGER, 2);
        axisMap.put(Axis.RIGHT_TRIGGER, 3);
        axisMap.put(Axis.RIGHT_HORIZONTAL, 4);
        axisMap.put(Axis.RIGHT_VERTICAL, 5);
    }
}
