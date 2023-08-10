package com.team1816.lib.controlboard;

import java.util.EnumMap;

/**
 * Serves as a base to define how the InputHandler interprets
 * the input from the available controller ports.
 *
 * @see InputHandler
 */
public class ControllerBinding {
    public final EnumMap<Button, Integer> buttonMap = new EnumMap<>(Button.class);

    public final EnumMap<Axis, Integer> axisMap = new EnumMap<>(Axis.class);

    public ControllerBinding() {}
}
