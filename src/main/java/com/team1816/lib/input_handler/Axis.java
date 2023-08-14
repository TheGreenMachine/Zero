package com.team1816.lib.input_handler;

/**
 * An abstraction to provide a consistent interface for users.
 */
public enum Axis {
    LEFT_HORIZONTAL,
    LEFT_VERTICAL,

    RIGHT_HORIZONTAL,
    RIGHT_VERTICAL,

    LEFT_TRIGGER,
    RIGHT_TRIGGER;

    public static final double axisThreshold = 0.04;
}
