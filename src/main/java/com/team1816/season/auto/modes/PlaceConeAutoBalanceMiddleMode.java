package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.NodeToChargeStationMiddlePath;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class PlaceConeAutoBalanceMiddleMode extends AutoMode {
    //Does NOT exit community

    public PlaceConeAutoBalanceMiddleMode() {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToChargeStationMiddlePath()
                )
            )
        );
    }

    public PlaceConeAutoBalanceMiddleMode(Color color) {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToChargeStationMiddlePath(color)
                )
            )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Place Cone Balance Mode");
        runAction(
            new SeriesAction(
                new WaitAction(0.25),
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.25),
                trajectoryActions.get(0),
                new WaitAction(0.5),
                new AutoBalanceAction()
            )
        );
    }
}
