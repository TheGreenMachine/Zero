package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.TrajectoryToPointAction;
import com.team1816.season.auto.paths.LivingRoomPath;

import java.util.List;

public class TrajectoryToPointMode extends AutoMode {
    public TrajectoryToPointMode() {
        super(List.of(new TrajectoryAction(new TrajectoryToPointAction())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(trajectoryActions.get(0));
    }
}
