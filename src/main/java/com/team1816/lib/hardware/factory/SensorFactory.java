package com.team1816.lib.hardware.factory;

import com.revrobotics.ColorSensorV3;
import com.team1816.lib.hardware.components.sensor.GhostColorSensor;
import com.team1816.lib.hardware.components.sensor.ProximitySensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

/**
 * This class like MotorFactory is the ultimate entry point for creating any and all sensors:
 * Color Sensors, Beam Breaks, Limit Switches, Hall Effects, etc.
 */
public class SensorFactory {

    private ColorSensorV3 createColorSensor(I2C.Port id, boolean ghost) {
        if (ghost) {
            var sensor = new GhostColorSensor(id);
            return sensor;
        } else {
            var sensor = new ColorSensorV3(id);
            configureColorSensorV3(sensor);
            return sensor;
        }
    }

    private ColorSensorV3 createColorSensor(I2C.Port id) {
        return createColorSensor(id, true);
    }

    private void configureColorSensorV3(ColorSensorV3 sensor) {
        configureColorSensorV3(
            sensor,
            ColorSensorV3.ColorSensorResolution.kColorSensorRes16bit,
            ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms,
            ColorSensorV3.GainFactor.kGain3x
        );
    }

    private void configureColorSensorV3(
        ColorSensorV3 sensor,
        ColorSensorV3.ColorSensorResolution res,
        ColorSensorV3.ColorSensorMeasurementRate mr,
        ColorSensorV3.GainFactor gf
    ) {
        sensor.configureColorSensor(res, mr, gf);
    }

    private DigitalInput createBeamBreakSensor(int id) {
        return new DigitalInput(id);
    }

    private DigitalInput createLimitSwitch(int id) {
        return new DigitalInput(id);
    }

    private ProximitySensor createProximitySensor(String NAME, int id) {
        return new ProximitySensor(NAME, id);
    }
}
