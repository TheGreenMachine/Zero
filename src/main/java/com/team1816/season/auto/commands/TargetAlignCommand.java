package com.team1816.season.auto.commands;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.ParallelAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitUntilInsideRegion;
import com.team1816.lib.auto.commands.AutoCommand;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import com.team1816.season.auto.actions.AlignAction;
import com.team1816.season.auto.paths.TargetTrajectoryPath;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public class TargetAlignCommand extends AutoCommand {

    private static RobotState robotState;
    private static LedManager ledManager;
    private Elevator.EXTENSION_STATE extensionState;

    public TargetAlignCommand(Elevator.EXTENSION_STATE extension) {
        super(List.of(new TrajectoryAction(new TargetTrajectoryPath())));
        robotState = Injector.get(RobotState.class);
        ledManager = Injector.get(LedManager.class);
        extensionState = extension;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Target Align Command!");
        if (robotState.allianceColor == Color.BLUE) {
            runAction(
                new ParallelAction(
                    trajectoryActions.get(0),
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                            new Translation2d(0, 0),
                            new Translation2d(3.20, Constants.fieldCenterY * 2),
                            "Blue Placement Region"
                        ),
                        new AlignAction(extensionState, 0, 0)
                    )
                )
            );
        } else {
            runAction(
                new ParallelAction(
                    trajectoryActions.get(0),
                    new SeriesAction(
                        new WaitUntilInsideRegion(
                            new Translation2d(13.03, 0),
                            new Translation2d(Constants.fieldCenterX * 2, Constants.fieldCenterY * 2),
                            "Red Placement Region"
                        ),
                        new AlignAction(extensionState, 0, 0)
                    )
                )
            );
        }
    }

    public void done() {
        super.done();
        Robot.runningAutoTargetAlign = false;
        ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.SOLID);
        GreenLogger.log("Target Align Command Completed!");
    }
}
