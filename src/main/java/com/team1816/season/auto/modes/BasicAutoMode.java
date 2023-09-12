package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.List;

public class BasicAutoMode extends AutoMode {
    public BasicAutoMode() {
        super(List.of(
            // Trajectory actions go here...
        ));
    }

    public BasicAutoMode(Color color) {
        super(List.of(
            // Trajectory actions go here (pass color here)...
        ));
    }

    // The routine will only be called once with .run()
    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Basic Auto Mode Action Routine");

        // Run actions here:
        // e.g. runAction(new SeriesAction(new WaitAction(0.5), ...))
    }
}
