package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.configuration.Constants;

public class AutoBalanceMode extends AutoMode {
    public AutoBalanceMode() {
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Auto Balance Mode");
        runAction(new AutoBalanceAction(Constants.kMaxBalancingVelocity));
        runAction(new WaitAction(.5));
    }
}
