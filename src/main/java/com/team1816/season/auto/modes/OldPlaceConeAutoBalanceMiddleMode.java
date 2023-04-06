package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.OldNodeToChargeStationMiddlePath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class OldPlaceConeAutoBalanceMiddleMode extends AutoMode {
    //Does NOT exit community

    public OldPlaceConeAutoBalanceMiddleMode() {
        super(
            List.of(
                new TrajectoryAction(
                    new OldNodeToChargeStationMiddlePath()
                )
            )
        );
    }

    public OldPlaceConeAutoBalanceMiddleMode(Color color) {
        super(
            List.of(
                new TrajectoryAction(
                    new OldNodeToChargeStationMiddlePath(color)
                )
            )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Place Cone Balance Mode");
        runAction(
            new SeriesAction(
                new WaitAction(0.25),
                new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.25),
                new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN),
                trajectoryActions.get(0),
                new WaitAction(0.1),
                new AutoBalanceAction()
            )
        );
    }
}
