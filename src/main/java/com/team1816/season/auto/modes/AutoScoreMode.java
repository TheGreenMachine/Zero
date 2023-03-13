package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.Robot;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.subsystems.Elevator;

import java.util.List;

public class AutoScoreMode extends AutoMode {
    private boolean isCube;
    private Elevator.EXTENSION_STATE extensionState;

    public AutoScoreMode(boolean isCube, Elevator.EXTENSION_STATE extensionState) {
        this.isCube = isCube;
        this.extensionState = extensionState;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Auto Score Mode!");
        runAction(new ScoreAction(isCube,extensionState));
    }

    public void done() {
        super.done();
        Robot.runningAutoScore = false;
        System.out.println("AutoScore ended");

    }
}
