package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AlignAction;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.auto.paths.ConeToNodeWallPath;
import com.team1816.season.auto.paths.NodeToConeWallPath;
import com.team1816.season.auto.paths.NodeToSecondConeWallPath;
import com.team1816.season.auto.paths.SecondConeToNodeWallPath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class TriplePlaceConeWallMode extends AutoMode {

    public TriplePlaceConeWallMode() {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToConeWallPath()
                ),
                new TrajectoryAction(
                    new ConeToNodeWallPath()
                ),
                new TrajectoryAction(
                    new NodeToSecondConeWallPath()
                ),
                new TrajectoryAction(
                    new SecondConeToNodeWallPath()
                )
            )
        );
    }

    public TriplePlaceConeWallMode(Color color) {
        super(
            List.of(
                new TrajectoryAction(
                    new NodeToConeWallPath(color)
                ),
                new TrajectoryAction(
                    new ConeToNodeWallPath(color)
                ),
                new TrajectoryAction(
                    new NodeToSecondConeWallPath(color)
                ),
                new TrajectoryAction(
                    new SecondConeToNodeWallPath(color)
                )
            )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Triple Place Cone Mode");
        runAction(
            new SeriesAction(
                // scoring first cone
                new SeriesAction(
                    new WaitAction(.05),
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
