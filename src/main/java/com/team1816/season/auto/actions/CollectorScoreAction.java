package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.states.Orchestrator;

public class CollectorScoreAction implements AutoAction {
    private Orchestrator orchestrator;
    public CollectorScoreAction () {
        orchestrator = Injector.get(Orchestrator.class);
    }
    @Override
    public void start() {
        orchestrator.setCollectorScoring(true);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {

    }
}
