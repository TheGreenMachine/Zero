package com.team1816.season.subsystems;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.AnalogInput;

public class ProxySensor extends Subsystem {

    private static AnalogInput sharp;

    private static AnalogInput sharpFront;

    private static final String NAME = "proxySensor";

    public ProxySensor(String name, Infrastructure inf, RobotState rs) {
        super(name, inf, rs);
        sharp = new gitAnalogInput((int)(Math.round((factory.getConstant("proxySensor")))));
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
