package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveOpenLoopAction;
import com.team1816.lib.auto.actions.WaitAction;

public class DriveStraightMode extends AutoMode {

    public DriveStraightMode() {
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Drive Straight Mode");
        runAction(new WaitAction(.5));
        runAction(new DriveOpenLoopAction(2, .25));
    }
}
