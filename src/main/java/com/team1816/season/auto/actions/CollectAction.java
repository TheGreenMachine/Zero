package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Collector;

public class CollectAction implements AutoAction {

    private RobotState robotState;
    private LedManager ledManager;
    private Collector collector;

    private boolean waitForPivot;

    private Collector.ROLLER_STATE desiredRollerState;

    private Collector.PIVOT_STATE desiredPivotState;

    public CollectAction(Collector.ROLLER_STATE rollerState) {
        robotState = Injector.get(RobotState.class);
        ledManager = Injector.get(LedManager.class);
        collector = Injector.get(Collector.class);
        this.desiredRollerState = rollerState;
        waitForPivot = true;
    }

    public CollectAction(Collector.PIVOT_STATE pivotState) {
        robotState = Injector.get(RobotState.class);
        ledManager = Injector.get(LedManager.class);
        collector = Injector.get(Collector.class);
        this.desiredPivotState = pivotState;
        waitForPivot = true;
    }

    public CollectAction(Collector.ROLLER_STATE rollerState, Collector.PIVOT_STATE pivotState) {
        robotState = Injector.get(RobotState.class);
        ledManager = Injector.get(LedManager.class);
        collector = Injector.get(Collector.class);
        this.desiredRollerState = rollerState;
        this.desiredPivotState = pivotState;
        waitForPivot = true;
    }

    public CollectAction(Collector.ROLLER_STATE rollerState, Collector.PIVOT_STATE pivotState, boolean waitForPivot) {
        robotState = Injector.get(RobotState.class);
        ledManager = Injector.get(LedManager.class);
        collector = Injector.get(Collector.class);
        this.desiredRollerState = rollerState;
        this.desiredPivotState = pivotState;
        this.waitForPivot = waitForPivot;
    }

    @Override
    public void start() {
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
        if (desiredRollerState != null) {
            GreenLogger.log("Setting collector to state: " + desiredRollerState.name());
            collector.setDesiredRollerState(desiredRollerState);
        }
        if (desiredPivotState != null) {
            GreenLogger.log("Setting collector to state: " + desiredPivotState.name() + ", Game element =  " + collector.getCurrentGameElement());
            collector.setDesiredPivotState(desiredPivotState);
        }
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if (!waitForPivot) {
            return true;
        } else {
            return robotState.actualCollectorPivotState.equals(collector.getDesiredPivotState());
        }
    }

    @Override
    public void done() {
        GreenLogger.log("Collect action completed");
    }
}
