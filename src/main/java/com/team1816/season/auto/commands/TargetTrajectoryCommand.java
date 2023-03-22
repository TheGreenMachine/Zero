package com.team1816.season.auto.commands;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.commands.AutoCommand;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.season.Robot;
import com.team1816.season.auto.paths.TargetTrajectoryPath;

import java.util.List;

public class TargetTrajectoryCommand extends AutoCommand {

    private static LedManager ledManager;

    public TargetTrajectoryCommand() {
        super(List.of(new TrajectoryAction(new TargetTrajectoryPath())));
        ledManager = Injector.get(LedManager.class);
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Trajectory To Command Mode!");
        runAction(trajectoryActions.get(0));
    }

    public void done() {
        super.done();
        Robot.runningAutoTarget = false;
        ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.SOLID);
        System.out.println("Trajectory To Target Command Completed!");
    }
}
