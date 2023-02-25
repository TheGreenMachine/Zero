package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.DriveFromCollectFeederPath;
import com.team1816.season.auto.paths.DriveToCollectFeederPath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class PlaceTwoFeederMode extends AutoMode {


    public PlaceTwoFeederMode() {
        super(List.of(new TrajectoryAction(new DriveToCollectFeederPath()), new TrajectoryAction(new DriveFromCollectFeederPath())));
    }
    public PlaceTwoFeederMode(Color color) {
        super(List.of(new TrajectoryAction(new DriveToCollectFeederPath(color)), new TrajectoryAction(new DriveFromCollectFeederPath(color))));
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(
            new SeriesAction(
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(.5),
                trajectoryActions.get(0),
                new CollectAction(Collector.STATE.INTAKE_CONE),
                new WaitAction(3),
                new CollectAction(Collector.STATE.STOP),
                trajectoryActions.get(1),
                new WaitAction(.5),
                new CollectAction(Collector.STATE.OUTTAKE_CONE),
                new WaitAction(1.2),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN)
            )
        );
    }


}
