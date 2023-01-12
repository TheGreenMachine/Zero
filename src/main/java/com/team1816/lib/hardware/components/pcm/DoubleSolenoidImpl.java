package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * This class is an implementation of a DoubleSolenoid (Solenoid that has two positions instead of the standard 1) and is a wrapper for
 * DoubleSolenoid and implements IDoubleSolenoid
 *
 * @see DoubleSolenoid
 * @see IDoubleSolenoid
 */
public class DoubleSolenoidImpl extends DoubleSolenoid implements IDoubleSolenoid {

    /**
     * Instantiates a DoubleSolenoidImpl
     *
     * @param moduleType     (CTREPCM, REVPH)
     * @param forwardChannel forward channel for module to use
     * @param reverseChannel reverse channel for module to use
     */
    public DoubleSolenoidImpl(
        final PneumaticsModuleType moduleType,
        final int forwardChannel,
        final int reverseChannel
    ) {
        super(moduleType, forwardChannel, reverseChannel);
    }

    /**
     * Instantiates a DoubleSolenoidImpl
     *
     * @param module         (solenoid module)
     * @param moduleType     (CTREPCM, REVPH)
     * @param forwardChannel forward channel for module to use
     * @param reverseChannel reverse channel for module to use
     */
    public DoubleSolenoidImpl(
        final int module,
        final PneumaticsModuleType moduleType,
        final int forwardChannel,
        final int reverseChannel
    ) {
        super(module, moduleType, forwardChannel, reverseChannel);
    }
}
