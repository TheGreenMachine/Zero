package com.team1816.lib.input_handler;

/**
 * An abstraction to provide a consistent interface for users.
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
    DOWN_RIGHT;

    public enum State {
        PRESSED,
        HOLD,
        RELEASED,
    }
}
