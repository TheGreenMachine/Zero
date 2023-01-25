package com.team1816.lib.hardware.components.sensor;

/**
 * This class is the main proximity sensor interface
 */
public interface IProximitySensor {
    String getName();

    double getVoltage();

    int getProximity();
}
