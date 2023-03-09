package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class that interfaces with a CANifier LedManager
 *
 * @see CANifier
 * @see ILEDManager
 */
public class CanifierImpl extends CANifier implements ILEDManager {


    private Color lastColor;

    /**
     * Instantiates a Canifier based on its device ID
     *
     * @param deviceId
     * @see CANifier
     */
    public CanifierImpl(int deviceId) {
        super(deviceId);
    }

    /**
     * Sets LEDs on a strip
     *
     * @param r        (red value 0-255)
     * @param g        (green value 0-255)
     * @param b        (blue value 0-255)
     * @param w        (white value)
     * @param startIdx (start ID of led to be controlled)
     * @param count    (number of LEDs from startIdx
     * @return ErrorCode / void
     * @see ILEDManager#setLEDs(int, int, int, int, int, int)
     */
    @Override
    public ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count) {
        lastColor = new Color(r, g, b);
        setLEDOutput(r / 255.0, CANifier.LEDChannel.LEDChannelB);
        setLEDOutput(g / 255.0, CANifier.LEDChannel.LEDChannelA);
        setLEDOutput(b / 255.0, CANifier.LEDChannel.LEDChannelC);
        return ErrorCode.OK;
    }

    /**
     * Functionality: non-existent
     *
     * @param b
     * @return ErrorCode / void
     * @see ILEDManager#configStatusLedState(boolean)
     */
    @Override
    public ErrorCode configStatusLedState(boolean b) {
        return null;
    }

    /**
     * Functionality: non-existent
     *
     * @param b
     * @return ErrorCode / void
     * @see ILEDManager#configLOSBehavior(boolean)
     */
    @Override
    public ErrorCode configLOSBehavior(boolean b) {
        return null;
    }

    /**
     * Functionality: non-existent
     *
     * @param brg
     * @return ErrorCode / void
     * @see ILEDManager#configLEDType(CANdle.LEDStripType)
     */
    @Override
    public ErrorCode configLEDType(CANdle.LEDStripType brg) {
        return null;
    }

    /**
     * Functionality: non-existent
     *
     * @param brightness
     * @return ErrorCode / void
     * @see ILEDManager#configBrightnessScalar(double)
     */
    @Override
    public ErrorCode configBrightnessScalar(double brightness) {
        return null;
    }

    /**
     * Functionality: non-existent
     *
     * @param animation
     * @return ErrorCode / void
     * @see ILEDManager#animate(Animation)
     */
    @Override
    public ErrorCode animate(Animation animation) {
        return null;
    }

    @Override
    public ErrorCode configV5Enabled(boolean enable5V, int timeoutMs) {
        return null;
    }

    @Override
    public Color getLastColor() {
        return lastColor;
    }
}
