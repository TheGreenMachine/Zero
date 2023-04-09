package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Elevator;

public class WaitUntilElevatorExtensionInsideRegion implements AutoAction {
    private static Elevator elevator;
    private final double minPos;
    private final double maxPos;

    public WaitUntilElevatorExtensionInsideRegion(double min, double max) {
        elevator = Injector.get(Elevator.class);

        minPos = Math.min(min, max) - elevator.getAllowableExtensionError();
        maxPos = Math.max(min, max) + elevator.getAllowableExtensionError();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return elevator.getActualExtensionTicks() > minPos && elevator.getActualExtensionTicks() < maxPos;
    }

    @Override
    public void done() {

    }
}
