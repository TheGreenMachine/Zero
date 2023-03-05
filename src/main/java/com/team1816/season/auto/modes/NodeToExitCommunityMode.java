package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.paths.NodeToExitCommunityPath;

import java.util.List;

public class NodeToExitCommunityMode extends AutoMode {

    public NodeToExitCommunityMode() {
        super(List.of(new TrajectoryAction(new NodeToExitCommunityPath())));
    }

    public NodeToExitCommunityMode(Color color) {
        super(List.of(new TrajectoryAction(new NodeToExitCommunityPath(color))));
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Move Out Mode");
        runAction(new WaitAction(.5));
        runAction(trajectoryActions.get(0));
    }
}
