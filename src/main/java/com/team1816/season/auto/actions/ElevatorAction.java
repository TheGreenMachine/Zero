package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.wpilibj.RobotBase;

import static com.team1816.lib.subsystems.Subsystem.robotState;

public class ElevatorAction implements AutoAction {

    private RobotState robotState;
    private Elevator elevator;
    private Elevator.ANGLE_STATE desiredAngleState;
    private Elevator.EXTENSION_STATE desiredExtensionState;

    private AsyncTimer simWaitTimer = new AsyncTimer(0.5, null); // just waits .5 secs b4 completing action

    public ElevatorAction(Elevator.ANGLE_STATE angle, Elevator.EXTENSION_STATE extension) {
        robotState = Injector.get(RobotState.class);
        elevator = Injector.get(Elevator.class);
        desiredAngleState = angle;
        desiredExtensionState = extension;
    }

    public ElevatorAction(Elevator.ANGLE_STATE angle, Elevator.EXTENSION_STATE extension, Collector.GAME_ELEMENT game_element) {
        robotState = Injector.get(RobotState.class);
        elevator = Injector.get(Elevator.class);
        desiredAngleState = angle;
        desiredExtensionState = extension;

        robotState.actualGameElement = game_element;
    }

    @Override
    public void start() {
        System.out.println("Setting elevator to angle: " + desiredAngleState.name() + " and extension to: " + desiredExtensionState.name());
        elevator.setDesiredState(desiredAngleState, desiredExtensionState);

        if (RobotBase.isSimulation()) {
            simWaitTimer.update();
        }
    }

    @Override
    public void update() {
        if (RobotBase.isSimulation()) {
            simWaitTimer.update();
        }
    }

    @Override
    public boolean isFinished() {
        if (RobotBase.isSimulation()) {
            return simWaitTimer.isCompleted();
        }

        return robotState.actualElevatorAngleState.equals(elevator.getDesiredAngleState())
            && robotState.actualElevatorExtensionState.equals(elevator.getDesiredExtensionState());
    }

    @Override
    public void done() {
        System.out.println("Elevator action completed: elevator angle at " + desiredAngleState.name() + " and extension at " + desiredExtensionState.name());
    }
}
