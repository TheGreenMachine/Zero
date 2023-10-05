package com.team1816.lib.input_handler;

public class XboxControllerBinding extends ControllerBinding {
    public XboxControllerBinding() {
        //TODO: Due to improper DriverStation simulation, these controls will be inaccurate in simulation if DS is not also open
        buttonMap.put(Button.A, 1);
        buttonMap.put(Button.B, 2);
        buttonMap.put(Button.X, 3);
        buttonMap.put(Button.Y, 4);
        buttonMap.put(Button.LEFT_BUMPER, 5);
        buttonMap.put(Button.RIGHT_BUMPER, 6);
        buttonMap.put(Button.BACK, 7);
        buttonMap.put(Button.START, 8);
        buttonMap.put(Button.LEFT_JOYSTICK, 9);
        buttonMap.put(Button.RIGHT_JOYSTICK, 10);
        axisMap.put(Axis.LEFT_HORIZONTAL, 0);
        axisMap.put(Axis.LEFT_VERTICAL, 1);

        axisMap.put(Axis.RIGHT_HORIZONTAL, 4);
        axisMap.put(Axis.RIGHT_VERTICAL, 5);

        triggerMap.put(Trigger.LEFT, 2);
        triggerMap.put(Trigger.RIGHT, 3);
    }
}
