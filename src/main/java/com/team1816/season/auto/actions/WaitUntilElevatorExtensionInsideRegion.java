package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.season.Robot;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.wpilibj.RobotBase;

public class WaitUntilElevatorExtensionInsideRegion implements AutoAction {
    private static Elevator elevator;
    private double minPos, maxPos;

    public WaitUntilElevatorExtensionInsideRegion(double min, double max) {
        elevator = Injector.get(Elevator.class);

        minPos = Math.min(min, max);
        maxPos = Math.max(min, max);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        if (RobotBase.isSimulation()){
            return true;
        }
        return elevator.getActualExtensionPosition() > minPos && elevator.getActualExtensionPosition() < maxPos;
    }

    @Override
    public void done() {

    }
}
