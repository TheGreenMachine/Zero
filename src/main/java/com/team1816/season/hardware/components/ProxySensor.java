package com.team1816.season.hardware.components;

import edu.wpi.first.wpilibj.AnalogInput;

public class ProxySensor {
    private AnalogInput sharp;

    private String NAME;

    public ProxySensor(String name, int port) {
        NAME = name;
        sharp = new AnalogInput(Math.round(port));
    }

    public double getVoltage() {
        double sensorVoltage = sharp.getVoltage();
        if(sensorVoltage > 4.0 || sensorVoltage < 0.0){
            sensorVoltage = 0.0;
        }
        return sensorVoltage;
    }

    public double getDistance() {
        return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
    }

    public String getName(){
        return NAME;
    }

}
