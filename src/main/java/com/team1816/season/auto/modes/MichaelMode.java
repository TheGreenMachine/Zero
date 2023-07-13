package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.paths.MichaelPath;
import com.team1816.lib.auto.Color;

import java.util.List;

public class MichaelMode extends AutoMode {
    public MichaelMode() {
        super(
                List.of(
                    new TrajectoryAction(
                            new MichaelPath()
                    )
                )
        );
    }

    public MichaelMode(Color color) {
        super(
                List.of(
                        new TrajectoryAction(
                                new MichaelPath(color)
                        )
                )
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("MichaelMode does something");

        if (robotState.actualCollectorAngle == 60.0) {
            throw new AutoModeEndedException();
        }
    }
}
