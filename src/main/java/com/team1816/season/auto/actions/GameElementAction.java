package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Collector;

public class GameElementAction implements AutoAction {

    private Collector collector;
    private Collector.GAME_ELEMENT gameElement;

    public GameElementAction(Collector.GAME_ELEMENT element) {
        collector = Injector.get(Collector.class);
        gameElement = element;
    }

    @Override
    public void start() {
        collector.setCurrentGameElement(gameElement);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return collector.getCurrentGameElement() == gameElement;
    }

    @Override
    public void done() {

    }
}
