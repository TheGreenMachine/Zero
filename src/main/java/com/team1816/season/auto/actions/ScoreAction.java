package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class ScoreAction extends SeriesAction {

    private static Elevator.EXTENSION_STATE level;

    public ScoreAction(Collector.GAME_ELEMENT gameElement, Elevator.EXTENSION_STATE extensionState) {
        super(
            new SeriesAction(
                new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.SCORE),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, extensionState),
                new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.SCORE),
                new CollectAction(gameElement == Collector.GAME_ELEMENT.CUBE ? Collector.ROLLER_STATE.OUTTAKE_CUBE : Collector.ROLLER_STATE.OUTTAKE_CONE),
                new WaitAction(gameElement == Collector.GAME_ELEMENT.CUBE ? .40 : .20), //For comp only, cube hotfix
                // resetting elevator / collector to starting states
                new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW, false),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN)
            )
        );

        level = extensionState;
    }

    @Override
    public void start() {
        GreenLogger.log("Started Score Action Sequence (Level: " + level.name() + ")");
        super.start();
    }

    @Override
    public void done() {
        GreenLogger.log("Score Action Sequence Completed (Level: " + level.name() + ")");
        super.done();
    }
}
