package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.season.Robot;
import com.team1816.season.auto.actions.ScoreAction;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class AutoScoreMode extends AutoMode {
    private Collector.GAME_ELEMENT gameElement;
    private Elevator.EXTENSION_STATE extensionState;

    public AutoScoreMode(Collector.GAME_ELEMENT gameElement, Elevator.EXTENSION_STATE extensionState) {
        this.gameElement = gameElement;
        this.extensionState = extensionState;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Auto Score Mode!");
        runAction(new ScoreAction(gameElement, extensionState));
    }

    public void done() {
        super.done();
        Robot.runningAutoScore = false;
        System.out.println("Auto Score Mode Completed!");
    }
}