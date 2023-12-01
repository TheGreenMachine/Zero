package com.team1816.lib.input_handler;

import java.util.HashMap;

/**
 * Primarily a class to hold data about how to map
 * an identifier from a yaml file to a specific button, axis,
 * trigger, or dpad.
 *
 * DO NOT USE THIS CLASS AT ANY POINT IN YOUR OWN CODE. It's use
 * is inherently limited to only the classes below:
 *
 * @see InputHandlerBridge
 * @see InputHandler
 */
class ControllerMappingInfo {
    public HashMap<String, Axis> joysticks;
    public HashMap<String, Button> buttons;
    public HashMap<String, Trigger> triggers;
    public HashMap<String, Dpad> dpad;

    public ControllerMappingInfo() {
        joysticks = new HashMap<>();
        buttons = new HashMap<>();
        triggers = new HashMap<>();
        dpad = new HashMap<>();
    }
}
