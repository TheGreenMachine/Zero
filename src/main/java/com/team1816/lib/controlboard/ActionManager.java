package com.team1816.lib.controlboard;

import com.team1816.lib.Injector;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.Arrays;
import java.util.List;

import static com.team1816.lib.controlboard.ControlUtils.createAction;
import static com.team1816.lib.controlboard.ControlUtils.createHoldAction;


/**
 * Responsible for executing specific actions associated with buttons, triggers, and joystick values represented in each controller
 *
 * @see ControlBoard
 * @see com.team1816.lib.controlboard.ControlBoardBridge
 * @see com.team1816.lib.controlboard.Controller
 */
public class ActionManager {

    private IControlBoard controlBoard;
    private Drive drive;

    private List<ControlUtils.ButtonAction> actions;

    //Quick and dirty way of getting limited input in disabled before input handler is in
    private List<ControlUtils.ButtonAction> disabledActions;

    public ActionManager(ControlUtils.ButtonAction... actions) {
        this.actions = Arrays.asList(actions);
        registerDisabledActions();
        this.update(); // Used to insure actions are in an initialized state and aren't prematurely triggered
        controlBoard = Injector.get(IControlBoard.class);
        drive = (Injector.get(Drive.Factory.class)).getInstance();
    }

    public void update() {
        actions.forEach(ControlUtils.ButtonAction::update);
    }

    public void updateDisabled() {
        disabledActions.forEach(ControlUtils.ButtonAction::update);
    }

    private void registerDisabledActions() {
        disabledActions = List.of(
                createAction(
                        () -> controlBoard.getAsBool("node2"),
                        () -> {
                            if (drive.gaudette.isPlaying()) {
                                drive.gaudette.pause();
                            } else {
                                drive.gaudette.play();
                            }
                        }
                )
        );
    }

}
