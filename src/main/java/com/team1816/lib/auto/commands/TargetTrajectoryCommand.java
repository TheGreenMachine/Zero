package com.team1816.lib.auto.commands;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.paths.TargetTrajectoryPath;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.states.Orchestrator;

import java.util.List;

public class TargetTrajectoryCommand extends AutoCommand {

    private static LedManager ledManager;

    public TargetTrajectoryCommand() {
        super(List.of(new TrajectoryAction(new TargetTrajectoryPath())));
        ledManager = Injector.get(LedManager.class);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Target Trajectory Command!");
        runAction(trajectoryActions.get(0));
    }

    public void done() {
        super.done();
        Orchestrator.runningAutoTarget = false;
        ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.SOLID);
        GreenLogger.log("Target Trajectory Command Completed!");
    }
}
