package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.AlignMaxAction;
import com.team1816.season.auto.actions.GameElementAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class PlaceConeMode extends AutoMode {

    public PlaceConeMode() {
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Place Cone Mode");
        runAction(new WaitAction(.5));
        runAction(
            new SeriesAction(
                new GameElementAction(Collector.GAME_ELEMENT.CONE),
                new AlignMaxAction(),
                new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MAX)
            )
        );
    }
}
