package com.team1816.lib.hardware.components.sensor;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This class represents an adapted Sharp proximity sensor implementation
 */
public class ProximitySensor implements IProximitySensor {
    private AnalogInput sensor;
    private String NAME;
    private PROXY_ORIENTATION sensorOrientation;


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
     * Initializes a proximity sensor given a name, its analog port on the RoboRIO, and its orientation
     *
     * @param name the name of the sensor
     * @param port the analog port that the sensor is wired to
     * @param orientation the orientation of the proximity sensor (Ex. Front left)
     */
    public ProximitySensor(String name, int port, PROXY_ORIENTATION orientation) {
        NAME = name;
        sensor = new AnalogInput(port);
        sensorOrientation = orientation;
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

    public enum PROXY_ORIENTATION{
        FRONT_LEFT(45),
        FRONT_RIGHT(135),
        BACK_LEFT(225),
        BACK_RIGHT(315);

        public final int proxyOrientOffset;

        PROXY_ORIENTATION(int proxyOrientOffset) {this.proxyOrientOffset = proxyOrientOffset;}

        public int getProxyOrientOffset(){
            return proxyOrientOffset;
        }




    }

}
