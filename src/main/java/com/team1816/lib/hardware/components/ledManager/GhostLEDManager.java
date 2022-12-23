package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

/**
 * This class emulates the behaviour of a LEDManager that is not physically implemented on a robot
 * @see ILEDManager
 */
public class GhostLEDManager implements ILEDManager {

    @Override
    public ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFactoryDefault() {
        return null;
    }

    @Override
    public ErrorCode configStatusLedState(boolean b) {
        return null;
    }

    @Override
    public ErrorCode configLOSBehavior(boolean b) {
        return null;
    }

    @Override
    public ErrorCode configLEDType(CANdle.LEDStripType brg) {
        return null;
    }

    @Override
    public ErrorCode configBrightnessScalar(double brightness) {
        return null;
    }

    @Override
    public ErrorCode animate(Animation animation) {
        return null;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        CANifierStatusFrame statusFrame,
        int periodMs,
        int timeoutMs
    ) {
        return ErrorCode.OK;
    }
}
