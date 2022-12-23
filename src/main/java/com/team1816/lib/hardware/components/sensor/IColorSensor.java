package com.team1816.lib.hardware.components.sensor;

import edu.wpi.first.wpilibj.util.Color;

/**
 * This is a universal interface for a Color Sensor
 */
public interface IColorSensor {
    int getId();

    Color getColor();

    int getRed();

    int getGreen();

    int getBlue();

    int[] getRGB();
}
