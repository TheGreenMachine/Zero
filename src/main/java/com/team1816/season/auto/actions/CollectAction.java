package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.season.subsystems.Collector;

public class CollectAction implements AutoAction {

    private LedManager ledManager;
    private Collector collector;

    private Collector.ROLLER_STATE desiredRollerState;

    private Collector.PIVOT_STATE desiredPivotState;


    public CollectAction(Collector.ROLLER_STATE rollerState, Collector.PIVOT_STATE pivotState) {
        ledManager = Injector.get(LedManager.class);
        collector = Injector.get(Collector.class);
        this.desiredRollerState = rollerState;
        this.desiredPivotState = pivotState;
    }

    @Override
    public void start() {
        System.out.println("Setting collector to state: " + desiredRollerState.name());
        if (desiredRollerState == Collector.ROLLER_STATE.INTAKE_CONE) {
            ledManager.indicateStatus(LedManager.RobotStatus.CONE, LedManager.ControlState.BLINK);
        } else if (desiredRollerState == Collector.ROLLER_STATE.INTAKE_CUBE) {
            ledManager.indicateStatus(LedManager.RobotStatus.CUBE, LedManager.ControlState.BLINK);
        } else if (desiredRollerState == Collector.ROLLER_STATE.OUTTAKE_CONE) {
            ledManager.indicateStatus(LedManager.RobotStatus.CONE, LedManager.ControlState.SOLID);
        } else if (desiredRollerState == Collector.ROLLER_STATE.OUTTAKE_CUBE) {
            ledManager.indicateStatus(LedManager.RobotStatus.CUBE, LedManager.ControlState.SOLID);
        } else if (desiredRollerState == Collector.ROLLER_STATE.STOP) {
            ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET);
        }
        collector.setDesiredState(desiredRollerState, desiredPivotState);
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
