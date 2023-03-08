package com.team1816.lib.hardware.components.ledManager;

import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The base interface that connects to all LEDManagers
 */
public interface ILEDManager {
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
     */
    ErrorCode setLEDs(int r, int g, int b, int w, int startIdx, int count);

    /**
     * Configures factory defaults
     *
     * @return ErrorCode / void
     */
    ErrorCode configFactoryDefault();

    /**
     * Configures status LED behaviour when other LEDs are being controlled
     *
     * @param b (disableWhenRunning)
     * @return ErrorCode / void
     */
    ErrorCode configStatusLedState(boolean b);

    /**
     * Configures LED behaviour if connection lost
     *
     * @param b (disableWhenLOS)
     * @return ErrorCode / void
     */
    ErrorCode configLOSBehavior(boolean b);

    /**
     * Configures the type of LED being controlled
     *
     * @param brg {@link CANdle.LEDStripType}
     * @return ErrorCode / void
     */
    ErrorCode configLEDType(CANdle.LEDStripType brg);

    /**
     * Configures the brightness scalar
     *
     * @param brightness (double [0, 1])
     * @return ErrorCode / void
     */
    ErrorCode configBrightnessScalar(double brightness);

    /**
     * Animates the LEDs based on an Animation
     *
     * @param animation (Animation)
     * @return ErrorCode / void
     * @see Animation
     */
    ErrorCode animate(Animation animation);

    ErrorCode configV5Enabled(boolean enable5V, int timeoutMs);

    /**
     * Sets the status frame period of the LEDManager
     *
     * @param statusFrame {@link CANifierStatusFrame}
     * @param periodMs    (period milliseconds)
     * @param timeoutMs   (timeout milliseconds)
     * @return ErrorCode / void
     * @see CANifierStatusFrame
     */
    ErrorCode setStatusFramePeriod(
        CANifierStatusFrame statusFrame,
        int periodMs,
        int timeoutMs
    );

    Color getLastColor();
}
