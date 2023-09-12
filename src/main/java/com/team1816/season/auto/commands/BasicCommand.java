package com.team1816.season.auto.commands;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.commands.AutoCommand;
import com.team1816.lib.util.logUtil.GreenLogger;

public class BasicCommand extends AutoCommand {
    /**
     * Properties
     */

    // Create as many constructors as you'd like.
    public BasicCommand() {

    }

    // The routine will only be called once with .run()
    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Basic Command once");

        // Run actions here...
    }
}
