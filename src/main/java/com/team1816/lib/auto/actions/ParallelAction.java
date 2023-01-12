package com.team1816.lib.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The ParallelAction class allows for a simultaneous composite action.
 * It runs all sub-actions at the same time.
 * All actions are started and updated until all actions are rendered as completed.
 * Can be used as a member of {@link SeriesAction}
 *
 * @see AutoAction
 */
public class ParallelAction implements AutoAction {

    /**
     * State: List of Actions to be completed in parallel
     */
    private final ArrayList<AutoAction> mActions;

    /**
     * Instantiates a parallel action based on the list of actions to be completed simultaneously
     *
     * @param actions
     */
    public ParallelAction(List<AutoAction> actions) {
        mActions = new ArrayList<>(actions);
    }

    /**
     * Alternative constructor that takes actions in a different format
     *
     * @param actions
     */
    public ParallelAction(AutoAction... actions) {
        this(Arrays.asList(actions));
    }

    /**
     * Starts all actions in mActions in an iterative manner
     *
     * @see AutoAction#start()
     */
    @Override
    public void start() {
        mActions.forEach(AutoAction::start);
    }

    /**
     * Updates all actions in mActions in an iterative manner, which allows for parallel execution
     *
     * @see AutoAction#update()
     */
    @Override
    public void update() {
        mActions.forEach(AutoAction::update);
    }

    /**
     * Returns true if all actions in mActions are finished, otherwise continues
     *
     * @return boolean isFinished
     * @see AutoAction#isFinished()
     */
    @Override
    public boolean isFinished() {
        for (AutoAction action : mActions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Standard verification cleanup for all actions in mActions
     *
     * @see AutoAction#done()
     */
    @Override
    public void done() {
        mActions.forEach(AutoAction::done);
    }
}
