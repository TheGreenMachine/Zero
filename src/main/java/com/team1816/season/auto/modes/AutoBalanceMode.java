package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.AutoBalanceAction;

public class AutoBalanceMode extends AutoMode {

    public AutoBalanceMode(){}
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new AutoBalanceAction(3));
    }
}
