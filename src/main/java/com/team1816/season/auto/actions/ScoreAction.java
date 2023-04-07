package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class ScoreAction extends SeriesAction {

    public ScoreAction(Collector.GAME_ELEMENT gameElement, Elevator.EXTENSION_STATE extensionState) {
        super(
            new SeriesAction(
                new CollectAction(Collector.ROLLER_STATE.STOP),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, extensionState),
                new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.SCORE),
                new CollectAction(gameElement == Collector.GAME_ELEMENT.CUBE ? Collector.ROLLER_STATE.OUTTAKE_CUBE : Collector.ROLLER_STATE.OUTTAKE_CONE),
                new WaitAction(.35),
                // resetting elevator / collector to starting states
                new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW, false),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN)
            )
        );
    }
}
