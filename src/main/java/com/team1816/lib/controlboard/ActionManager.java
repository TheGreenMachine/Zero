package com.team1816.lib.controlboard;

import java.util.Arrays;
import java.util.List;

/**
 * Responsible for executing specific actions associated with buttons, triggers, and joystick values represented in each controller
 * @see ControlBoard
 * @see com.team1816.lib.controlboard.ControlBoardBridge
 * @see com.team1816.lib.controlboard.Controller
 */
public class ActionManager {

    private List<ControlUtils.ButtonAction> actions;

    public ActionManager(ControlUtils.ButtonAction... actions) {
        this.actions = Arrays.asList(actions);
        this.update(); // Used to insure actions are in an initialized state and aren't prematurely triggered
    }

    public void update() {
        actions.forEach(ControlUtils.ButtonAction::update);
    }
}
