package com.team1816.lib.events;

import com.team1816.lib.controlboard.Controller;

/**
 * A base event to facilitate passing a controller around
 *
 * @see com.team1816.lib.controlboard.ControlBoard.DriverControllerEvent
 * @see com.team1816.lib.controlboard.ControlBoard.OperatorControllerEvent
 * @see com.team1816.lib.controlboard.ControlBoard.ButtonBoardControllerEvent
 */
public class ControllerEvent extends PubSubConsumer<Controller> {}
