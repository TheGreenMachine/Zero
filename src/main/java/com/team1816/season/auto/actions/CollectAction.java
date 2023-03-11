package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Collector;

public class CollectAction implements AutoAction {

    private Collector collector;

    private Collector.ROLLER_STATE desiredROLLERState;


    public CollectAction(Collector.ROLLER_STATE colROLLERState) {
        collector = Injector.get(Collector.class);
        this.desiredROLLERState = colROLLERState;
    }

    @Override
    public void start() {
        System.out.println("Setting collector to state: " + desiredROLLERState.name());
        collector.setDesiredState(desiredROLLERState);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
