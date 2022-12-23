package com.team1816.lib.hardware.components.sensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This class emulates the values of a color sensor that is not physically implemented on a robot
 */
public class GhostColorSensor extends ColorSensorV3 implements IColorSensor {

    public GhostColorSensor(I2C.Port port) {
        super(port);
    }

    @Override
    public int getId() {
        return 0;
    }

    @Override
    public Color getColor() {
        return new Color(0, 0, 0);
    }

    @Override
    public int[] getRGB() {
        return new int[0];
    }
}
