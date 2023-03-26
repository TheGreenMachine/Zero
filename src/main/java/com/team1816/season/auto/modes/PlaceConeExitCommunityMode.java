package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.NodeToExitCommunityPath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class PlaceConeExitCommunityMode extends AutoMode {

    public PlaceConeExitCommunityMode() {
        super(List.of(new TrajectoryAction(new NodeToExitCommunityPath())));
    }

    public PlaceConeExitCommunityMode(Color color) {
        super(List.of(new TrajectoryAction(new NodeToExitCommunityPath(color))));
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Move Out Mode");
        runAction(new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MAX));
        runAction(new WaitAction(.5));
        runAction(trajectoryActions.get(0));
    }
}
