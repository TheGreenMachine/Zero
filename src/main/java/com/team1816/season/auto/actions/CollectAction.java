package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Collector;

public class CollectAction implements AutoAction {

    private Collector collector;

    private Collector.ROLLER_STATE desiredRollerState;

    private Collector.PIVOT_STATE desiredPivotState;


    public CollectAction(Collector.ROLLER_STATE rollerState, Collector.PIVOT_STATE pivotState) {
        collector = Injector.get(Collector.class);
        this.desiredRollerState = rollerState;
        this.desiredPivotState = pivotState;
    }

    @Override
    public void start() {
        System.out.println("Setting collector to state: " + desiredRollerState.name());
        collector.setDesiredState(desiredRollerState, desiredPivotState);
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
