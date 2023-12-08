package com.team1816.lib.input_handler.bindings;

import com.team1816.lib.input_handler.InputHandler;
import com.team1816.lib.input_handler.controlOptions.Axis;
import com.team1816.lib.input_handler.controlOptions.Button;
import com.team1816.lib.input_handler.controlOptions.Trigger;

import java.util.EnumMap;

/**
 * Serves as a base to define how the InputHandler interprets
 * the input from the available controller ports.
 *
 * @see InputHandler
 */
public abstract class ControllerBinding {
    public EnumMap<Button, Integer> buttonMap = new EnumMap<>(Button.class);

    public EnumMap<Axis, Integer> axisMap = new EnumMap<>(Axis.class);

    public EnumMap<Trigger, Integer> triggerMap = new EnumMap<>(Trigger.class);
}
