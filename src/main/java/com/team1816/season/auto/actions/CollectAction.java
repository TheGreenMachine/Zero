package com.team1816.season.auto.actions;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class CollectAction implements AutoAction {

    private Collector collector;

    private Collector.STATE desiredState;


    public CollectAction(Collector.STATE colState){
        collector = Injector.get(Collector.class);
        this.desiredState = colState;
    }

    @Override
    public void start() {
        collector.setDesiredState(desiredState);
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
