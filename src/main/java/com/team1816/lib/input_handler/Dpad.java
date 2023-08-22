package com.team1816.lib.input_handler;

/**
 * Dpad values
 */
public enum Dpad {
    UP(0),
    UP_RIGHT(45),
    RIGHT(90),
    DOWN_RIGHT(135),
    DOWN(180),
    DOWN_LEFT(225),
    LEFT(270),
    UP_LEFT(315),
    CENTER(-1);

    public enum State {
        PRESSED,
        HELD,
        RELEASED,
    }

    public int value;

    Dpad(int value) {
        this.value = value;
    }
}
