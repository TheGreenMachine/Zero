package com.team1816.lib.input_handler.bindings;

import com.team1816.lib.input_handler.controlOptions.Button;

public class ButtonBoardControllerBinding extends ControllerBinding {
    public ButtonBoardControllerBinding() {
        buttonMap.put(Button.UP_LEFT, 2);
        buttonMap.put(Button.UP, 5);
        buttonMap.put(Button.UP_RIGHT, 12);
        buttonMap.put(Button.LEFT, 3);
        buttonMap.put(Button.CENTER, 4);
        buttonMap.put(Button.RIGHT, 10);
        buttonMap.put(Button.DOWN_LEFT, 9);
        buttonMap.put(Button.DOWN, 11);
        buttonMap.put(Button.DOWN_RIGHT, 13);
    }
}
