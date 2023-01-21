package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * This class is a lightweight wrapper implementation of a Solenoid and implements ISolenoid
 *
 * @see Solenoid
 * @see ISolenoid
 */
public class SolenoidImpl extends Solenoid implements ISolenoid {

    /**
     * Instantiates a SolenoidImpl
     *
     * @param moduleType (CTREPCM, REVPH)
     * @param channel    pneumatic channel
     */
    public SolenoidImpl(final PneumaticsModuleType moduleType, final int channel) {
        super(moduleType, channel);
    }

    /**
     * Instantiates a SolenoidImpl
     *
     * @param module     module
     * @param moduleType (CTREPCM, REVPH)
     * @param channel    pneumatic channel
     */
    public SolenoidImpl(
        final int module,
        final PneumaticsModuleType moduleType,
        final int channel
    ) {
        super(module, moduleType, channel);
    }
}
