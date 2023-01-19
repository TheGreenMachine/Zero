package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.ErrorCode;

/**
 * The base interface for all motor sensors
 */
public interface IMotorSensor {
    int getQuadraturePosition();

    int getPulseWidthPosition();

    ErrorCode setQuadraturePosition(int newPosition);
}
