package com.team1816.lib.hardware.components.sensor;

/**
 * This class represents a ghosted proximity sensor
 */
public class GhostProximitySensor implements IProximitySensor {

    public GhostProximitySensor() {
    }

    @Override
    public String getName() {
        return "null";
    }

    @Override
    public double getVoltage() {
        return 0;
    }

    @Override
    public int getProximity() {
        return 0;
    }
}
