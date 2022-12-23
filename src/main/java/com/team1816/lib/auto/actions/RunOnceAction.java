package com.team1816.lib.auto.actions;

/**
 * Template action for something that only needs to be done once and has no need for updates.
 * @see Action
 */
public abstract class RunOnceAction implements Action {

    /**
     * Single start call, conducts the action
     * @see Action#start()
     */
    @Override
    public void start() {
        runOnce();
    }

    /**
     * Does not exist in this case / is left empty
     */
    @Override
    public void update() {}

    /**
     * Determines weather or not the action is finished
     * @return true if action start has been called
     * @see Action#isFinished()
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Standard verification cleanup the action
     * @see Action#done()
     */
    @Override
    public void done() {}

    /**
     * Abstract instantiation
     */
    public abstract void runOnce();
}
