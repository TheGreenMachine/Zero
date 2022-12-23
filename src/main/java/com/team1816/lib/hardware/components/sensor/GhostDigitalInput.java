package com.team1816.lib.hardware.components.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class emulates the behaviour of a DigitalInput based sensor that is not physically implemented on a robot
 */
public class GhostDigitalInput extends DigitalInput {

    //states
    private boolean on;

    @Override
    public boolean get() {
        return on;
    }

    @Override
    public int getChannel() {
        return -1;
    }

    public GhostDigitalInput(int channel) {
        super(channel);
    }
}
