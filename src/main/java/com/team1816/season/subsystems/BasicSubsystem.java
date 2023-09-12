package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

public class BasicSubsystem extends Subsystem {
    /**
     * Properties
     */
    public static final String NAME = "Basic Subsystem";

    private boolean outputsChanged;
    private boolean inputsChanged;

    /**
     * Subsystem Description:
     * The purpose of this subsystem is to show you the basic layout of
     * how a subsystem should be implemented.
     */
    @Inject
    public BasicSubsystem(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);

        outputsChanged = false;
        inputsChanged = false;
    }

    public void writeToHardware() {
        if (outputsChanged) {
            // Logic to run if values have been updated and you need
            // to write to the hardware
        }
    }

    public void readFromHardware() {
        if (inputsChanged) {
            // Logic to run if values need to be updated based on a
            // condition else where in the code.
        }
    }

    public void zeroSensors() {
        // Logic to run if you need to zero out (or set to a default) any
        // sensors or motors you may be using.
    }

    public void stop() {

    }

    public boolean testSubsystem() {
        // Insert logic to test the state of the subsystem.

        return true;
    }
}
