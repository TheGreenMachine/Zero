package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;

/**
 * A class that interfaces with the CANdle LedManager
 *
 * @see CANdle
 * @see ILEDManager
 */
public class CANdleImpl extends CANdle implements ILEDManager {

    /**
     * Instantiates a CANdle based on CAN-bus id and CAN-bus name
     *
     * @param candle          (CAN-bus ID)
     * @param canivoreBusName (CAN-bus name)
     * @see CANdle
     */
    public CANdleImpl(Integer candle, String canivoreBusName) {
        super(candle, canivoreBusName);
    }

    /**
     * Functionality: non-existent
     *
     * @param statusFrame {@link CANifierStatusFrame}
     * @param periodMs    period (milliseconds)
     * @param timeoutMs   timeout (milliseconds)
     * @return ErrorCode / void
     * @see ILEDManager#setStatusFramePeriod(CANifierStatusFrame, int, int)
     */
    @Override
    public ErrorCode setStatusFramePeriod(
        CANifierStatusFrame statusFrame,
        int periodMs,
        int timeoutMs
    ) {
        return null;
    }
}
