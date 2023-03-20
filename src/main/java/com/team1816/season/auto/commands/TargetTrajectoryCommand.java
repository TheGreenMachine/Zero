package com.team1816.season.auto.commands;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.Robot;
import com.team1816.season.auto.paths.TargetTrajectoryPath;

import java.util.List;

public class TargetTrajectoryCommand extends AutoMode {

    public TargetTrajectoryCommand() {
        super(List.of(new TrajectoryAction(new TargetTrajectoryPath())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Trajectory To Command Mode!");
        runAction(trajectoryActions.get(0));
    }

    public void done() {
        super.done();
        Robot.runningAutoTarget = false;
        System.out.println("Trajectory To Target Command Completed!");
    }
}
