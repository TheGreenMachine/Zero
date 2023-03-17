package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class AlignAction extends SeriesAction {

    public AlignAction(Elevator.EXTENSION_STATE extensionState) {
        super(
            // extension to desired scoring level
            new ElevatorAction(Elevator.ANGLE_STATE.SCORE, extensionState),
            // outtaking the game piece
            new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.SCORE)
        );
    }
}
