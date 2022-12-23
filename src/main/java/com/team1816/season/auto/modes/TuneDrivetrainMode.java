package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.auto.paths.DriveStraightPath;
import java.util.List;

public class TuneDrivetrainMode extends AutoMode {

    public TuneDrivetrainMode() {
        super(List.of(new TrajectoryAction(new DriveStraightPath())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Tune Drivetrain Mode");
        runAction(new WaitAction(.5));
        runAction(trajectoryActions.get(0));
    }
}
