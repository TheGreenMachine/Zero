package com.team1816.lib.hardware.components.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This class represents an adapted Sharp proximity sensor implementation
 */
public class ProximitySensor implements IProximitySensor {
    private AnalogInput sensor;
    private String NAME;

    /**
     * Initializes a proximity sensor given a name and its analog port on the RoboRIO
     *
     * @param name the name of the sensor
     * @param port the analog port that the sensor is wired to
     */
    public ProximitySensor(String name, int port) {
        NAME = name;
        sensor = new AnalogInput(port);
    }

    /**
     * Returns the voltage of the sensor
     *
     * @return sensorVoltage
     */
    public double getVoltage() {
        double sensorVoltage = sensor.getVoltage();
        if (sensorVoltage > 4.0 || sensorVoltage < 0.0) {
            sensorVoltage = 0.0;
        }
        return sensorVoltage;
    }

    /**
     * Returns the voltage based proximity via the manufacture specified mapping
     *
     * @return proximity
     */
    public int getProximity() {
        return (int) ((Math.pow(sensor.getAverageVoltage(), -1.2045)) * 27.726);
    }

    /**
     * Returns the name of the sensor
     *
     * @return name
     */
    public String getName() {
        return NAME;
    }

}
