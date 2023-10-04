package com.team1816.lib.input_handler;

/**
 * A virtual Axis mapping to Button mapping.
 */
public enum Trigger {
    LEFT,
    RIGHT;

    public enum State {
        PRESSED,
        HELD,
        RELEASED,
    }

    public static final double kAxisThreshold = 0.04;
}
