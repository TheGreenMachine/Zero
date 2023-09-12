package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.AutoAction;

public class BasicAction implements AutoAction {
    /**
     * Properties
     */

    // Make as many constructors as you need...
    public BasicAction() {

    }

    @Override
    public void start() {
        // logic to call at the start of calling this action.
        // this method is usually used to set up state.
    }

    @Override
    public void update() {
        // Insert any update logic here that will run before the action is finished.
    }

    @Override
    public boolean isFinished() {
        // Implement logic for it to check if appropriate state is set up.
        return true;
    }

    @Override
    public void done() {
        // Logic to execute when the action is finished.
    }


}
