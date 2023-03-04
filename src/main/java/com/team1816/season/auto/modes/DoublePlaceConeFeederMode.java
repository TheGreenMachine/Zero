package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.*;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.ConeToNodeFeederPath;
import com.team1816.season.auto.paths.NodeToConeFeederPath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Translation2d;

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
        runAction(
            new SeriesAction(
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.5),
                new ParallelAction(
                    trajectoryActions.get(0),
                    new SeriesAction(
                        new WaitAction(1),
                        new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN),
                        new CollectAction(Collector.STATE.INTAKE_CONE),
                        new WaitAction(2),
                        new CollectAction(Collector.STATE.STOP)
                    )
                ),
                new ElevatorAction(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN),
                trajectoryActions.get(1),
                new WaitAction(0.25),
                new ScoreAction(false, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.5)
            )
        );
    }


}
