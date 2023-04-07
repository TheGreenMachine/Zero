package com.team1816.season.auto.commands;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.commands.AutoCommand;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import com.team1816.season.auto.actions.AlignAction;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class AutoScoreCommand extends AutoCommand {
    private Collector.GAME_ELEMENT gameElement;
    private Elevator.EXTENSION_STATE extensionState;

    public AutoScoreCommand(Collector.GAME_ELEMENT gameElement, Elevator.EXTENSION_STATE extensionState) {
        this.gameElement = gameElement;
        this.extensionState = extensionState;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        GreenLogger.log("Running Auto Score Command!");
        runAction(new ScoreAction(gameElement, extensionState));
    }

    public void done() {
        super.done();
        Robot.runningAutoScore = false;
        GreenLogger.log("Auto Score Mode Command!");
    }
}