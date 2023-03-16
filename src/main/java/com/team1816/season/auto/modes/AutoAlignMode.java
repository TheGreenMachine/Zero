package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.Robot;
import com.team1816.season.auto.actions.AlignAction;
import com.team1816.season.subsystems.Elevator;

public class AutoAlignMode extends AutoMode {
    private Elevator.EXTENSION_STATE extensionState;

    public AutoAlignMode(Elevator.EXTENSION_STATE extensionState) {
        this.extensionState = extensionState;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Auto Score Mode!");
        runAction(new AlignAction(extensionState));
    }

    public void done() {
        super.done();
        Robot.runningAutoAlign = false;
        System.out.println("Auto Score Mode Completed!");
    }
}