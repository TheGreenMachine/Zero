package com.team1816.lib.loops;

import com.team1816.lib.subsystems.SubsystemLooper;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is a synchronized registry of various loops that can be running.
 *
 * @see SubsystemLooper
 * @see Robot (disabledLoop and enabledLoop)
 */
public class Looper implements ILooper {

    /**
     * State
     */
    private boolean mRunning;
    private final List<Loop> mLoops;
    private final Object mTaskRunningLock = new Object();
    private double mTimestamp = 0;
    private double mDT = 0;
    private double mStart = 0;

    /**
     * Instantiates a looper and ties it to the iterative periodic robot base
     *
     * @param robot
     * @see TimedRobot
     */
    public Looper(TimedRobot robot) {
        Runnable runnable_ = new Runnable() {
            @Override
            public void run() {
                synchronized (mTaskRunningLock) {
                    if (mRunning) {
                        double now = Timer.getFPGATimestamp();

                        for (Loop loop : mLoops) {
                            loop.onLoop(now);
                        }

                        mDT = now - mTimestamp;
                        mTimestamp = now;
                    }
                }
            }
        };

        robot.addPeriodic(runnable_, Constants.kLooperDt, 0.005); // offsets periodic loop
        mRunning = false;
        mLoops = new ArrayList<>();
    }

    /**
     * Synchronously adds a loop to {@link Looper#mLoops} in the registry
     *
     * @param loop
     */
    @Override
    public synchronized void register(Loop loop) {
        synchronized (mTaskRunningLock) {
            mLoops.add(loop);
        }
    }

    /**
     * Starts all loops that are attached to the looper
     */
    public synchronized void start() {
        synchronized (mTaskRunningLock) {
            if (!mRunning) {
                GreenLogger.log("Starting loops");
                mTimestamp = Timer.getFPGATimestamp();
                mStart = mTimestamp;
                for (Loop loop : mLoops) {
                    loop.onStart(mTimestamp);
                }
                mRunning = true;
            }
        }
    }

    /**
     * Stops all loops that are running on the looper
     */
    public synchronized void stop() {
        synchronized (mTaskRunningLock) {
            if (mRunning) {
                GreenLogger.log("Stopping loops");
                mRunning = false;
                mTimestamp = Timer.getFPGATimestamp();
                for (Loop loop : mLoops) {
                    loop.onStop(mTimestamp);
                }
            }
        }
    }

    /**
     * Returns the millisecond timestamp of the last loop that was run
     *
     * @return lastLoop
     */
    public double getLastLoop() {
        if (!mRunning) return 0;
        return mDT * 1000; // Convert to ms
    }

    public boolean isRunning() {
        return mRunning;
    }
}
