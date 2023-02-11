package com.team1816.season.subsystems;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

public class Elevator extends Subsystem {
    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param name String
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    public Elevator(String name, Infrastructure inf, RobotState rs) {
        super(name, inf, rs);
    }

    @Override
    public void readFromHardware() {

    }

    @Override
    public void writeToHardware() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean testSubsystem() {
        return false;
    }
}
