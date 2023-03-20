package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class PlaceConeMode extends AutoMode {

    public PlaceConeMode() {
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Place Cone Mode");
        runAction(new WaitAction(.5));
        runAction(new ScoreAction(Collector.GAME_ELEMENT.CONE, Elevator.EXTENSION_STATE.MAX));
    }
}
