package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Elevator;

public class ElevatorAction implements AutoAction {

    private final RobotState robotState;
    private final Elevator elevator;

    private Elevator.ANGLE_STATE initialAngleState;
    private Elevator.EXTENSION_STATE initialExtensionState;
    private final Elevator.ANGLE_STATE desiredAngleState;
    private final Elevator.EXTENSION_STATE desiredExtensionState;
    private boolean minMaxTransitionTriggered = false;

    public ElevatorAction(Elevator.ANGLE_STATE angle, Elevator.EXTENSION_STATE extension) {
        robotState = Injector.get(RobotState.class);
        elevator = Injector.get(Elevator.class);

        desiredAngleState = angle;
        desiredExtensionState = extension;
    }

    @Override
    public void start() {
        GreenLogger.log("Setting elevator to angle: " + desiredAngleState.name() + " and extension to: " + desiredExtensionState.name());
        initialAngleState = robotState.actualElevatorAngleState;
        initialExtensionState = robotState.actualElevatorExtensionState;

        if (initialExtensionState != desiredExtensionState) {
            if (initialExtensionState == Elevator.EXTENSION_STATE.MAX) { // transition at mid extension
                elevator.setDesiredState(initialAngleState, Elevator.EXTENSION_STATE.MID);
                minMaxTransitionTriggered = false;
            } else if (desiredExtensionState == Elevator.EXTENSION_STATE.MAX) { // transition at mid extension
                elevator.setDesiredState(desiredAngleState, Elevator.EXTENSION_STATE.MID);
                minMaxTransitionTriggered = false;
            }
        } else {
            elevator.setDesiredState(desiredAngleState, desiredExtensionState);
            minMaxTransitionTriggered = true;
        }
    }

    @Override
    public void update() {
        if (!minMaxTransitionTriggered) {
            if (elevator.extensionAtTarget(8)) {
                elevator.setDesiredState(desiredAngleState, desiredExtensionState);
                minMaxTransitionTriggered = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return robotState.actualElevatorAngleState.equals(desiredAngleState)
            && robotState.actualElevatorExtensionState.equals(desiredExtensionState);
    }

    @Override
    public void done() {
        GreenLogger.log("Elevator action completed: elevator angle at " + desiredAngleState.name() + " and extension at " + desiredExtensionState.name());
    }
}
