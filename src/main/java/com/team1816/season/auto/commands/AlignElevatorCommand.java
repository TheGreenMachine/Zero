package com.team1816.season.auto.commands;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.commands.AutoCommand;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import com.team1816.season.auto.actions.AlignAction;
import com.team1816.season.subsystems.Elevator;

public class AlignElevatorCommand extends AutoCommand {
    private Elevator.EXTENSION_STATE extensionState;

    public AlignElevatorCommand(Elevator.EXTENSION_STATE extensionState) {
        this.extensionState = extensionState;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Auto Score Command!");
        if (extensionState == Elevator.EXTENSION_STATE.MAX) {
            runAction(new AlignAction(extensionState, Elevator.EXTENSION_STATE.MID.getExtension(), Elevator.EXTENSION_STATE.MAX.getExtension()));
        } else if (extensionState == Elevator.EXTENSION_STATE.MID) {
            runAction(new AlignAction(extensionState, (Elevator.EXTENSION_STATE.MIN.getExtension() + Elevator.EXTENSION_STATE.MID.getExtension()) / 2d, Elevator.EXTENSION_STATE.MID.getExtension()));
        } else {
            runAction(new AlignAction(extensionState, 0, (Elevator.EXTENSION_STATE.MIN.getExtension() + Elevator.EXTENSION_STATE.MID.getExtension()) / 2d));
        }
    }

    public void done() {
        super.done();
        Robot.runningAutoAlign = false;
        GreenLogger.log("Auto Score Command Completed!");
    }
}