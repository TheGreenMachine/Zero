package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.auto.actions.DriveOpenLoopAction;
import com.team1816.season.auto.paths.DriveStraightPath;
import com.team1816.season.auto.paths.LivingRoomPath;

import java.util.List;

public class AutoBalanceMode extends AutoMode {

    public AutoBalanceMode(){
        super(List.of(new TrajectoryAction(new DriveStraightPath(2))));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(.5));
        runAction(trajectoryActions.get(0));
        runAction(new AutoBalanceAction());
    }
}
