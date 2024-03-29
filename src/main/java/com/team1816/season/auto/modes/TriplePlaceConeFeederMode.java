package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.*;
import com.team1816.season.auto.paths.ConeToNodeFeederPath;
import com.team1816.season.auto.paths.NodeToConeFeederPath;
import com.team1816.season.auto.paths.NodeToSecondConeFeederPath;
import com.team1816.season.auto.paths.SecondConeToNodeFeederPath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class TriplePlaceConeFeederMode extends AutoMode {

    public TriplePlaceConeFeederMode() {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToConeFeederPath()
                ),
                new TrajectoryAction(
                    new ConeToNodeFeederPath()
                ),
                new TrajectoryAction(
                    new NodeToSecondConeFeederPath()
                ),
                new TrajectoryAction(
                    new SecondConeToNodeFeederPath()
                )
            )
        );
    }

    public TriplePlaceConeFeederMode(Color color) {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToConeFeederPath(color)
                ),
                new TrajectoryAction(
                    new ConeToNodeFeederPath(color)
                ),
                new TrajectoryAction(
                    new NodeToSecondConeFeederPath(color)
                ),
                new TrajectoryAction(
                    new SecondConeToNodeFeederPath(color)
                )
            )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Triple Place Cone Mode");
        runAction(
            new SeriesAction(
                new GameElementAction(Collector.GAME_ELEMENT.CONE),
                // scoring first cone
                new SeriesAction(
                    new WaitAction(.05),
                    new AlignMinAction(),
                    new ParallelAction(
                        new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MIN),
                        new SeriesAction(
                            new WaitAction(1),
                            new ParallelAction(
                                trajectoryActions.get(0),
                                new SeriesAction(
                                    new WaitAction(1),
                                    new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MID),
                                    new CollectAction(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.FLOOR)
                                )
                            )
                        )
                    )
                ),
                // collecting second cone
                new SeriesAction(
                    new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MID),
                    new CollectAction(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.FLOOR),
                    new WaitAction(0.25)
                ),
                // aligning
                new ParallelAction(
                    new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN),
                    trajectoryActions.get(1),
                    new SeriesAction(
                        new WaitAction(1),
                        new AlignAction(Elevator.EXTENSION_STATE.MIN, 0, 0)
                    )
                ),
                // scoring second cone
                new SeriesAction(
                    new WaitAction(.05),
                    new ParallelAction(
                        new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MIN),
                        new SeriesAction(
                            new WaitAction(1),
                            new ParallelAction(
                                trajectoryActions.get(2),
                                new SeriesAction(
                                    new WaitAction(1),
                                    new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MID),
                                    new CollectAction(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.FLOOR)
                                )
                            )
                        )
                    )
                ),
                // collecting third cone
                new SeriesAction(
                    new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MID),
                    new CollectAction(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.FLOOR),
                    new WaitAction(0.25)
                ),
                // aligning
                new ParallelAction(
                    new ElevatorAction(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN),
                    trajectoryActions.get(3),
                    new SeriesAction(
                        new WaitAction(1),
                        new AlignAction(Elevator.EXTENSION_STATE.MIN, 0, 0)
                    )
                ),
                // scoring third cone
                new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(0.25)
            )
        );
    }
}
