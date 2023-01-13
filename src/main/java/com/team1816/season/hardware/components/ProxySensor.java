package com.team1816.season.hardware.components;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.AnalogInput;

public class ProxySensor {

    private static AnalogInput sharp;

    private static AnalogInput sharpFront;

    private static final String NAME = "proxySensor";

    public ProxySensor(String name, int id) {
        sharp = new AnalogInput((int)(Math.round(id)));
    }
}
