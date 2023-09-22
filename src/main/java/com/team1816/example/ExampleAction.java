package com.team1816.example;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;

public class ExampleAction implements AutoAction {
    /**
     * Properties
     */
    public ExampleSubsystem exSub;
    public ExampleSubsystem.STATE desiredState;

    /**
     * Create a constructor that takes in any necessary values you need.
     * @param desState
     */
    public ExampleAction(ExampleSubsystem.STATE desState) {
        this.exSub = Injector.get(ExampleSubsystem.class);
        desiredState = desState;
    }

    /**
     * A method used to initialize any state or values at the beginning of
     * this action being used.
     */
    @Override
    public void start() {
        exSub.stop();
    }

    /**
     * A method that gets called repeatedly until the action is finished.
     */
    @Override
    public void update() {

    }

    /**
     * A method that determines if this action is complete.
     * @return boolean
     */
    @Override
    public boolean isFinished() {
        // Insert any necessary logic to ensure that the action completes
        // successfully.
        return true;
    }

    /**
     * A method that gets called once the action is finished.
     */
    @Override
    public void done() {

    }
}
