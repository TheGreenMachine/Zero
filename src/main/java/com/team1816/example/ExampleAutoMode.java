package com.team1816.example;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.List;

public class ExampleAutoMode extends AutoMode {

    /**
     * The main way to construct an auto mode.
     *
     * Passes in a list of trajectory actions for the mode to complete.
     */
    public ExampleAutoMode() {
        super(List.of(new TrajectoryAction(new ExampleAutoPath())));
    }

    /**
     * Secondary constructor for AutoMode.
     *
     * Some actions require Color information and this constructor
     * allows you to pass that in.
     * @param color
     */
    public ExampleAutoMode(Color color) {
        super(List.of(new TrajectoryAction(new ExampleAutoPath())));
    }

    /**
     * This method only gets called once when the auto mode is ran with
     * .run()
     * @throws AutoModeEndedException
     */
    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Auto Mode Routine Begin");

        // Run actions here:
        // e.g. runAction(new SeriesAction(new WaitAction(0.5), ...))
        runAction(
            new SeriesAction(
                new WaitAction(0.5),
                trajectoryActions.get(0)
            )
        );
    }
}
