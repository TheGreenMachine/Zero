package com.team1816.lib.loops;

import edu.wpi.first.wpilibj.Timer;
import javax.annotation.Nullable;

/**
 * This is a timer that allows for asynchronous timing separate from a loop i.e. it will not halt a thread
 */
public class AsyncTimer {

    /**
     * State
     */
    private double startTime;
    private double duration;
    private Runnable startAction;
    private Runnable endAction;
    private boolean hasStarted;
    private boolean completed;

    /**
     * Instantiates an AsyncTimer
     * @param duration duration to time / wait
     * @param startAction starting trigger
     * @param endAction end response
     */
    public AsyncTimer(
        double duration,
        @Nullable Runnable startAction,
        Runnable endAction
    ) {
        this.duration = duration;
        this.startAction = startAction;
        this.endAction = endAction;
        this.hasStarted = false;
        this.completed = false;
    }

    /**
     * Alternatively instantiates an AsyncTimer with only an ending action
     * @param duration duration to time / wait
     * @param endAction end response
     */
    public AsyncTimer(double duration, Runnable endAction) {
        this(duration, null, endAction);
    }

    /**
     * Updates the time based on a logical operation which allows for the loop to not be halted.
     * In waiting period, does nothing
     */
    public void update() {
        if (completed) return;
        if (!hasStarted) {
            startTime = Timer.getFPGATimestamp(); // Timer.getFPGATimeStamp in SECONDS
            if (startAction != null) {
                startAction.run();
            }
            hasStarted = true;
        } else {
            if (Timer.getFPGATimestamp() >= startTime + duration) {
                completed = true;
                endAction.run();
            }
        }
    }

    /**
     * Returns the status of the timer
     * @return {@link AsyncTimer#completed}
     */
    public boolean isCompleted() {
        return completed;
    }

    /**
     * Resets the timer
     */
    public void reset() {
        completed = false;
        hasStarted = false;
        startTime = 0;
    }
}
