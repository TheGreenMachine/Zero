package com.team1816.lib.input_handler;

import java.util.EnumMap;

/**
 * Serves as a base to define how the InputHandler interprets
 * the input from the available controller ports.
 *
 * @see InputHandler
 */
public abstract class ControllerBinding {
    EnumMap<Button, Integer> buttonMap = new EnumMap<>(Button.class);

    EnumMap<Axis, Integer> axisMap = new EnumMap<>(Axis.class);

    EnumMap<Trigger, Integer> triggerMap = new EnumMap<>(Trigger.class);
}
