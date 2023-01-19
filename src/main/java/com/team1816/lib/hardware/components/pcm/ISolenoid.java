package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.Sendable;

/**
 * This class is the universal interface for a Solenoid.
 *
 * @see edu.wpi.first.wpilibj.Solenoid
 */
public interface ISolenoid extends Sendable, AutoCloseable {
    boolean get();

    void set(boolean on);

    void toggle();
}
