package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Compressor;

/**
 * This class is a universal interface for a Compressor
 *
 * @see edu.wpi.first.wpilibj.Compressor
 */
public interface ICompressor extends Sendable, AutoCloseable {
    /**
     * Enables the compressor to accept closed loop Digital Input
     *
     * @see Compressor#enableDigital()
     */
    void enableDigital();

    /**
     * Disables the compressor
     *
     * @see Compressor#disable()
     */
    void disable();

    /**
     * Returns the state of the compressor enabled or disabled
     *
     * @return boolean enabled
     */
    boolean enabled();

    /**
     * Returns the compressor current
     *
     * @return double current
     * @see Compressor#getCurrent()
     */
    double getCompressorCurrent();

    /**
     * Returns the compressor pressure switch value
     *
     * @return boolean switchValue
     * @see Compressor#getPressureSwitchValue()
     */
    boolean getCompressorSwitchValue();
}
