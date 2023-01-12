package com.team1816.example;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;

public class ExampleAction implements AutoAction {

    public ExampleSubsystem exSub;
    public ExampleSubsystem.STATE desiredState;

    public ExampleAction(ExampleSubsystem.STATE desState) {
        this.exSub = Injector.get(ExampleSubsystem.class);
        desiredState = desState;
    }

    @Override
    public void start() {
        exSub.stop();
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
