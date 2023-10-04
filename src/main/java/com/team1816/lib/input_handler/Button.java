package com.team1816.lib.input_handler;

/**
 * A virtual Button mapping.
 */
public enum Button {
    A,
    B,
    X,
    Y,
    LEFT_BUMPER,
    RIGHT_BUMPER,
    BACK,
    START,
    LEFT_JOYSTICK,
    RIGHT_JOYSTICK,

    // Button Board
    CENTER,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    UP_LEFT,
    UP_RIGHT,
    DOWN_LEFT,
    DOWN_RIGHT;

    public enum State {
        PRESSED,
        HELD,
        RELEASED,
    }
}
