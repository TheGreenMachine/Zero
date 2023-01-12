package com.team1816.lib.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * The SeriesAction class executes a set of actions in a series.
 * Each action is started updated after the previous action is complete.
 * Can be used as a member of {@link ParallelAction}
 *
 * @see AutoAction
 */

public class SeriesAction implements AutoAction {

    /**
     * State: Temporary state variable for the current action
     */
    private AutoAction mCurrentAction;
    /**
     * State: Queue of remaining actions yet to be completed
     */
    private final ArrayList<AutoAction> mRemainingActions;

    /**
     * Instantiates a series action based on the list of actions to be completed sequentially
     *
     * @param actions
     */
    public SeriesAction(List<AutoAction> actions) {
        mRemainingActions = new ArrayList<>(actions);
        mCurrentAction = null;
    }

    /**
     * Alternative constructor that takes actions in a different format
     *
     * @param actions
     */
    public SeriesAction(AutoAction... actions) {
        this(Arrays.asList(actions));
    }

    /**
     * Starts the sequence of actions that consist of the series action
     *
     * @see AutoAction#start()
     */
    @Override
    public void start() {
    }

    /**
     * Updates the current action
     * If the current action is null / empty, a new action is assigned from the queue of remaining actions.
     * If the queue of remaining actions is empty, nothing happens, the action is complete.
     * Otherwise, iterates through the current action
     *
     * @see AutoAction#update()
     */
    @Override
    public void update() {
        if (mCurrentAction == null) {
            if (mRemainingActions.isEmpty()) {
                return;
            }

            mCurrentAction = mRemainingActions.remove(0);
            mCurrentAction.start();
        }

        mCurrentAction.update();

        if (mCurrentAction.isFinished()) {
            mCurrentAction.done();
            mCurrentAction = null;
        }
    }

    /**
     * Returns whether or not the series action is complete based on if the remaining actions are empty and so is the current action
     *
     * @return boolean isFinished
     * @see AutoAction#isFinished()
     */
    @Override
    public boolean isFinished() {
        return mRemainingActions.isEmpty() && mCurrentAction == null;
    }

    /**
     * Standard verification cleanup for the series action
     *
     * @see AutoAction#done()
     */
    @Override
    public void done() {
    }
}
