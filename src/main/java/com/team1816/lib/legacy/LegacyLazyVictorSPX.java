package com.team1816.lib.legacy;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is a thin wrapper around the VictorSPX that reduces CAN bus / CPU overhead.
 * Connects with CTRE VictorSPX motor controllers and adapts it for the universal IGreenMotor.
 *
 * @see IGreenMotor
 * @see VictorSPX
 */
@Deprecated
public class LegacyLazyVictorSPX extends VictorSPX implements LegacyIGreenMotor {

    protected String name = "";

    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public LegacyLazyVictorSPX(int deviceNumber, String motorName) {
        super(deviceNumber);
        name = motorName;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration currLimitCfg,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: configSupplyCurrentLimit not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrameEnhanced frame,
        int periodMs,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: setStatusFramePeriod not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public double getOutputCurrent() {
        DriverStation.reportWarning(
            "method: getOutputCurrent not implemented for LazyVictorSPX",
            false
        );
        return 0;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        DriverStation.reportWarning(
            "method: configReverseLimitSwitchSource not implemented for LazyVictorSPX",
            false
        );
        return null;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs) {
        GreenLogger.log(
            "WARNING: configAllSettings not implemented in LazyVictorSPX!"
        );
        return ErrorCode.OK;
    }
}
