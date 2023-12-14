package com.team1816.lib.legacy;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;

/**
 * The base universal interface for all motors
 */
@Deprecated
public interface LegacyIGreenMotor extends IMotorControllerEnhanced {
    String getName();

    ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs);

    ErrorCode configFactoryDefault(int timeoutMs);

    double getOutputCurrent();
}
