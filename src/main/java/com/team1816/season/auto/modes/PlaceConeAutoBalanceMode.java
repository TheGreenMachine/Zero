package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.DriveToChargeStationPath;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class PlaceConeAutoBalanceMode extends AutoMode {

    public PlaceConeAutoBalanceMode() {
        super(List.of(new TrajectoryAction(new DriveToChargeStationPath())));
    }

    public PlaceConeAutoBalanceMode(Color color) {
        super(List.of(new TrajectoryAction(new DriveToChargeStationPath(color))));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running DriveToBalance_BalanceMode AutoMode");
        runAction(
            new SeriesAction(
                new WaitAction(.5),
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(.5),
                trajectoryActions.get(0),
                new AutoBalanceAction()
            )
        );

    }
}