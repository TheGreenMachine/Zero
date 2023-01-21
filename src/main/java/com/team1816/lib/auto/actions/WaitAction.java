package com.team1816.lib.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 *
 * @see WaitAction
 */
public class WaitAction implements AutoAction {

    /**
     * State: duration of action
     */
    private final double mTimeToWait;
    /**
     * State: start time of action
     */
    private double mStartTime;

    /**
     * Instantiates a WaitAction with a time to stop
     *
     * @param timeToWait
     */
    public WaitAction(double timeToWait) {
        mTimeToWait = timeToWait;
    }

    /**
     * Records start time and begins the action
     *
     * @see AutoAction#start()
     */
    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    /**
     * Nonexistant
     *
     * @see AutoAction#update()
     */
    @Override
    public void update() {
    }

    /**
     * Checks if the duration to wait has passed
     *
     * @return isFinished
     * @see AutoAction#isFinished()
     */
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait;
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
