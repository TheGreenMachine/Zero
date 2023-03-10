package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.NodeToExitCommunityPath;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class ExitCommunityPlaceWallMode extends AutoMode {

    public ExitCommunityPlaceWallMode() {
        super(List.of(new TrajectoryAction(new NodeToExitCommunityPath())));
    }

    public ExitCommunityPlaceWallMode(Color color) {
        super(List.of(new TrajectoryAction(new NodeToExitCommunityPath(color))));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Place Cone Exit Community Mode");
        runAction(
            new SeriesAction(
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.5),
                trajectoryActions.get(0)
            )
        );
    }
}
