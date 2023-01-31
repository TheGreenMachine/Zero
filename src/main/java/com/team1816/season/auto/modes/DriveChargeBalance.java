package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.auto.paths.DriveToChargePath;

import java.util.List;

public class DriveChargeBalance extends AutoMode {
    public DriveChargeBalance(){
        super(List.of(new TrajectoryAction(new DriveToChargePath())));
    }
    public DriveChargeBalance(Color color){
        super(List.of(new TrajectoryAction(new DriveToChargePath(color))));
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive to Charge Mode");
        runAction(new WaitAction(.5));
        runAction(trajectoryActions.get(0));
        runAction(new AutoBalanceAction(.4));
    }

}
