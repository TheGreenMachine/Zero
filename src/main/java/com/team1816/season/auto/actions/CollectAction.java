package com.team1816.season.auto.actions;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;

public class CollectAction implements AutoAction {

    private Collector collector;

    private Collector.PIVOT_STATE desiredPivotState;

    private ControlMode desiredControlMode;

    Collector.ROLLER_STATE desiredRollerState;



    public CollectAction(ControlMode controlMode, Collector.PIVOT_STATE desiredPivotState, Collector.ROLLER_STATE desiredRollerState){
        collector = Injector.get(Collector.class);
        this.desiredControlMode = controlMode;
        this.desiredPivotState = desiredPivotState;
        this.desiredRollerState = desiredRollerState;
    }

    @Override
    public void start() {
        collector.setDesiredState(desiredControlMode, desiredPivotState, desiredRollerState);
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
