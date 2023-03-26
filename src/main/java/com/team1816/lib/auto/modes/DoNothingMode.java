package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.util.logUtil.GreenLogger;

/**
 * Autonomous mode to do absolutely nothing
 */
public class DoNothingMode extends AutoMode {

    /**
     * Routine. Does nothing.
     *
     * @throws AutoModeEndedException
     * @see AutoMode#routine()
     */
    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("doing nothing");
    }
}
