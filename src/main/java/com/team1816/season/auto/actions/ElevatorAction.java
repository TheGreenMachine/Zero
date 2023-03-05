package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.subsystems.Elevator;

public class ElevatorAction implements AutoAction {

    private Elevator elevator;

    private Elevator.ANGLE_STATE desiredAngleState;

    private Elevator.EXTENSION_STATE desiredExtensionState;

    public ElevatorAction(Elevator.ANGLE_STATE angle, Elevator.EXTENSION_STATE extension) {
        elevator = Injector.get(Elevator.class);
        desiredAngleState = angle;
        desiredExtensionState = extension;
    }

    @Override
    public void start() {
        System.out.println("Setting elevator to angle: " + desiredAngleState.name() + " and extension to: " + desiredExtensionState.name());
        elevator.setDesiredState(desiredAngleState, desiredExtensionState);
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
