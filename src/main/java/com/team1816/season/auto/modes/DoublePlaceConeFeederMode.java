package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.ConeToNodeFeederPath;
import com.team1816.season.auto.paths.NodeToConeFeederPath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class DoublePlaceConeFeederMode extends AutoMode {

    public DoublePlaceConeFeederMode() {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToConeFeederPath()
                ),
                new TrajectoryAction(
                    new ConeToNodeFeederPath()
                )
            )
        );
    }

    public DoublePlaceConeFeederMode(Color color) {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToConeFeederPath(color)
                ),
                new TrajectoryAction(
                    new ConeToNodeFeederPath(color)
                )
            )
        );
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Double Place Cone Mode");
        runAction(
            new SeriesAction(
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.5),
                new ParallelAction(
                    trajectoryActions.get(0),
                    new SeriesAction(
                        new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN),
                        new CollectAction(Collector.ROLLER_STATE.INTAKE_CONE),
                        new WaitAction(3),
                        new CollectAction(Collector.ROLLER_STATE.STOP)
                    )
                ),
                trajectoryActions.get(1),
                new WaitAction(0.25),
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.5)
            )
        );
    }


}
