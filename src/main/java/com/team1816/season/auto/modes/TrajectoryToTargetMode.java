package com.team1816.season.auto.modes;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.TrajectoryToTargetPath;
import com.team1816.season.states.RobotState;

import java.util.List;

public class TrajectoryToTargetMode extends AutoMode {

    public TrajectoryToTargetMode() {
        super(List.of(new TrajectoryAction(new TrajectoryToTargetPath())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Trajectory To Target Mode");
        runAction(trajectoryActions.get(0));
    }
}
