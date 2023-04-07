package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class AlignAction extends SeriesAction {

    public AlignAction(Elevator.EXTENSION_STATE extensionState, double minCollectTriggerThreshold, double maxCollectTriggerThreshold) {
        super(
            // extension to desired scoring level
            new ParallelAction(
                new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, extensionState),
                // outtaking the game piece
                new SeriesAction(
                    new WaitUntilElevatorExtensionInsideRegion(minCollectTriggerThreshold, maxCollectTriggerThreshold),
                    new CollectAction(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.SCORE)
                )
            )
        );
    }

    @Override
    public void start() {
        GreenLogger.log("Started Elevator Align Action");
        super.start();
    }

    @Override
    public void done() {
        GreenLogger.log("Elevator Align Action Completed");
        super.done();
    }
}
