package com.team1816.lib.input_handler;

/**
 * a virtual Axis mapping.
 */
public enum Axis {
    LEFT_HORIZONTAL,
    LEFT_VERTICAL,

    RIGHT_HORIZONTAL,
    RIGHT_VERTICAL,

    LEFT_TRIGGER,
    RIGHT_TRIGGER;

    public static final double kAxisThreshold = 0.04;
}
