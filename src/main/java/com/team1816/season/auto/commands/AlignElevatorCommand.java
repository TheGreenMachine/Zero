package com.team1816.season.auto.commands;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.commands.AutoCommand;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.actions.AlignMaxAction;
import com.team1816.season.auto.actions.AlignMidAction;
import com.team1816.season.auto.actions.AlignMinAction;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.subsystems.Elevator;

public class AlignElevatorCommand extends AutoCommand {
    private Elevator.EXTENSION_STATE extensionState;

    public AlignElevatorCommand(Elevator.EXTENSION_STATE extensionState) {
        this.extensionState = extensionState;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Align Elevator Command!");
        if (extensionState == Elevator.EXTENSION_STATE.MAX) {
            runAction(new AlignMaxAction());
        } else if (extensionState == Elevator.EXTENSION_STATE.MID) {
            runAction(new AlignMidAction());
        } else {
            runAction(new AlignMinAction());
        }
    }

    public void done() {
        super.done();
        Orchestrator.runningAutoAlign = false;
        GreenLogger.log(" Align Elevator Command Completed!");
    }
}