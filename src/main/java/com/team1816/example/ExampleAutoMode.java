package com.team1816.example;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;

import java.util.List;

public class ExampleAutoMode extends AutoMode {

    public ExampleAutoMode() {
        super(List.of(new TrajectoryAction(new ExampleAutoPath())));
    }

    @Override
    protected void routine() throws AutoModeEndedException {

    }
}
