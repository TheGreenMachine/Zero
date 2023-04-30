package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.DriveOpenLoopAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.NodeToChargeStationMiddlePath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class PlaceCubeExitCommunityAutoBalanceMiddleMode extends AutoMode {
    public PlaceCubeExitCommunityAutoBalanceMiddleMode() {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToChargeStationMiddlePath()
                )
            )
        );
    }

    public PlaceCubeExitCommunityAutoBalanceMiddleMode(Color color) {
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
        GreenLogger.log("Running Place Cone Balance Mode");
        runAction(
            new SeriesAction(
                new GameElementAction(Collector.GAME_ELEMENT.CUBE),
                new WaitAction(0.25),
                new AlignMaxAction(),
                new ScoreAction(Collector.GAME_ELEMENT.CUBE, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.25),
                new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN),
                trajectoryActions.get(0),
                new DriveOpenLoopAction(1.4, .35),
                new DriveOpenLoopAction(1.4, -.35),
                new WaitAction(0.5),
                new AutoBalanceAction()
            )
        );
    }
}
