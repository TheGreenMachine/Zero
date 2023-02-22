package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.CollectorScoreAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class AutoScoreMode extends AutoMode {
    private Orchestrator.SCORE_LEVEL_STATE desiredScoreLevelState;

    public AutoScoreMode(Orchestrator.SCORE_LEVEL_STATE desiredScoreLevelState) {
        this.desiredScoreLevelState = desiredScoreLevelState;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        if (desiredScoreLevelState == Orchestrator.SCORE_LEVEL_STATE.MIN) {
            runAction(
                new SeriesAction(
                    new WaitAction(.5),
                    new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN),
                    new WaitAction(2),
                    new CollectorScoreAction(),
                    new WaitAction(.5),
                    new CollectAction(Collector.STATE.STOP),
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN),
                    new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN)
                )
            );
        } else if (desiredScoreLevelState == Orchestrator.SCORE_LEVEL_STATE.MID) {
            runAction(
                new SeriesAction(
                    new WaitAction(.5),
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MID),
                    new WaitAction(2),
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE_DIP, Elevator.EXTENSION_STATE.MID),
                    new WaitAction(.25),
                    new CollectorScoreAction(),
                    new WaitAction(.5),
                    new CollectAction(Collector.STATE.STOP),
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN),
                    new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN)
                )
            );
        } else {
            runAction(
                new SeriesAction(
                    new WaitAction(.5),
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MAX),
                    new WaitAction(2),
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE_DIP, Elevator.EXTENSION_STATE.MAX),
                    new WaitAction(.25),
                    new CollectorScoreAction(),
                    new WaitAction(.5),
                    new CollectAction(Collector.STATE.STOP),
                    new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN),
                    new WaitAction(.5),
                    new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN)
                )
            );
        }
    }
}
