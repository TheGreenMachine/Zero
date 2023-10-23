package com.team1816.lib.legacy;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is a thin wrapper around the TalonFX that reduces CAN bus / CPU overhead.
 * Connects with CTRE TalonFX motor controllers and adapts it for the universal IGreenMotor.
 *
 * @see IGreenMotor
 * @see TalonFX
 */
@Deprecated
public class LegacyLazyTalonFX extends TalonFX implements LegacyIGreenMotor {

    protected double lastSet = Double.NaN;
    protected String name = "";
    protected ControlMode lastControlMode = null;

    public LegacyLazyTalonFX(int deviceNumber, String motorName, String canBus) {
        super(deviceNumber, canBus);
        name = motorName;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != lastSet || mode != lastControlMode) {
            if (!super.hasResetOccurred()) {
                lastSet = value;
                lastControlMode = mode;
                super.set(mode, value);
            } else {
                DriverStation.reportError("MOTOR " + getDeviceID() + " HAS RESET", false);
            }
        }
    }

    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs) {
        return super.configAllSettings(allConfigs, timeoutMs);
    }

    @Override
    public double getOutputCurrent() {
        return super.getStatorCurrent();
    }
}
