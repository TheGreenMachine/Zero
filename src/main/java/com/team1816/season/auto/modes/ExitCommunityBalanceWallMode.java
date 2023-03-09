package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.auto.paths.NodeToChargeStationFeederPath;
import com.team1816.season.auto.paths.NodeToChargeStationWallPath;

import java.util.List;

public class ExitCommunityBalanceWallMode extends AutoMode {
    //Does NOT exit community

    public ExitCommunityBalanceWallMode() {
        super(List.of(new TrajectoryAction(new NodeToChargeStationFeederPath())));
    }

    public ExitCommunityBalanceWallMode(Color color) {
        super(List.of(new TrajectoryAction(new NodeToChargeStationWallPath(color))));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Exit Community Balance Mode");
        runAction(
            new SeriesAction(
                new WaitAction(.5),
                trajectoryActions.get(0),
                new AutoBalanceAction()
            )
        );

    }
}
