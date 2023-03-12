package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class ScoreAction extends SeriesAction {

    public ScoreAction(Collector.GAME_ELEMENT gameElement, Elevator.EXTENSION_STATE extensionState) {
        super(
            // extension to desired scoring level
            new ElevatorAction(Elevator.ANGLE_STATE.SCORE, extensionState),
            new WaitAction(2),
            // dipping and dropping game piece
            new ElevatorAction(Elevator.ANGLE_STATE.SCORE_DIP, extensionState),
            new WaitAction(.25),
            new CollectAction(gameElement == Collector.GAME_ELEMENT.CUBE ? Collector.ROLLER_STATE.OUTTAKE_CUBE : Collector.ROLLER_STATE.OUTTAKE_CONE, Collector.PIVOT_STATE.SCORE),
            new WaitAction(.5),
            // resetting elevator / collector to starting states
            new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW),
            new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN),
            new WaitAction(.5)
        );
    }
}
