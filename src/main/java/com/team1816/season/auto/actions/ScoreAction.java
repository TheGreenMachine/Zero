package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class ScoreAction extends SeriesAction {
    public ScoreAction(Collector.GAME_ELEMENT gameElement, Elevator.EXTENSION_STATE extensionState) {
        super(
            new SeriesAction(
                new ParallelAction(
                    // extension to desired scoring level
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE, extensionState, gameElement),
                    // outtaking the game piece
                    new CollectAction(Collector.PIVOT_STATE.SCORE)
                ),
                new WaitAction(.20),
                new CollectAction(gameElement == Collector.GAME_ELEMENT.CUBE ? Collector.ROLLER_STATE.OUTTAKE_CUBE : Collector.ROLLER_STATE.OUTTAKE_CONE),
                new WaitAction(.35),
                // resetting elevator / collector to starting states
                new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW, false),
                new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN)
            )
        );
    }

    @Override
    public void start() {
        GreenLogger.log("Started Score Action Sequence");
        super.start();
    }

    @Override
    public void done() {
        GreenLogger.log("Score Action Sequence Completed");
        super.done();
    }
}
