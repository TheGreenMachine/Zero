package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.ChargeStationToField;
import com.team1816.season.auto.paths.FieldToChargeStation;
import com.team1816.season.auto.paths.NodeToChargeStationMiddlePath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class PlaceConeExitCommunityAutoBalanceMiddleMode extends AutoMode {

    public PlaceConeExitCommunityAutoBalanceMiddleMode() {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToChargeStationMiddlePath()
                ),
                new TrajectoryAction(
                    new ChargeStationToField()
                ), new TrajectoryAction(
                    new FieldToChargeStation()
                )
            )
        );
    }

    public PlaceConeExitCommunityAutoBalanceMiddleMode(Color color) {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToChargeStationMiddlePath(color)
                ),
                new TrajectoryAction(
                    new ChargeStationToField(color)
                ), new TrajectoryAction(
                    new FieldToChargeStation(color)
                )
            )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Place Cone Balance Mode");
        runAction(
            new SeriesAction(
                new GameElementAction(Collector.GAME_ELEMENT.CONE),
                new WaitAction(0.25),
                new AlignMaxAction(),
                new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.25),
                new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN),
                trajectoryActions.get(0),
                new WaitAction(0.25),
                trajectoryActions.get(1),
                new WaitAction(0.25),
                trajectoryActions.get(2),
                new WaitAction(0.25),
                new AutoBalanceAction()
            )
        );
    }
}
