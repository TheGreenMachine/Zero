package com.team1816.lib.input_handler.bindings;

import com.team1816.lib.input_handler.controlOptions.Axis;
import com.team1816.lib.input_handler.controlOptions.Button;

public class WasdControllerBinding extends ControllerBinding {
    public WasdControllerBinding() {
        super();
        buttonMap.put(Button.A, 2);
        buttonMap.put(Button.B, 3);
        buttonMap.put(Button.X, 1);
        buttonMap.put(Button.Y, 4);
        buttonMap.put(Button.LEFT_BUMPER, 5);
        buttonMap.put(Button.RIGHT_BUMPER, 6);
        buttonMap.put(Button.BACK, 9);
        buttonMap.put(Button.START, 10);
        buttonMap.put(Button.LEFT_JOYSTICK, 11);
        buttonMap.put(Button.RIGHT_JOYSTICK, 12);

        axisMap.put(Axis.RIGHT_HORIZONTAL, 0);

        axisMap.put(Axis.LEFT_HORIZONTAL, 1);
        axisMap.put(Axis.LEFT_VERTICAL, 2);
    }
}
