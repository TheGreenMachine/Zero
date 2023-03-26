package com.team1816.lib.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.paths.LivingRoomPath;
import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.List;

public class LivingRoomMode extends AutoMode {

    private final boolean reflected = false;

    public LivingRoomMode() {
        super(List.of(new TrajectoryAction(new LivingRoomPath())));
    }

    public LivingRoomMode(Color color) {
        super(List.of(new TrajectoryAction(new LivingRoomPath(color))));

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Living Room Mode");
        runAction(new WaitAction(.5));
        runAction(trajectoryActions.get(0));
    }
}
