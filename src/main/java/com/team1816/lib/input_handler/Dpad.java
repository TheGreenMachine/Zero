package com.team1816.lib.input_handler;

/**
 * Dpad values
 */
public enum Dpad {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270),
    CENTER(-1);

    public int value;

    Dpad(int value) {
        this.value = value;
    }
}
