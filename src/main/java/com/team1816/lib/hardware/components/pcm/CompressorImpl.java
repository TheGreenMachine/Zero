package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * This is an implementation class for the Compressor
 *
 * @see Compressor
 * @see ICompressor
 */
public class CompressorImpl extends Compressor implements ICompressor {

    /**
     * Instantiates a CompressorImpl
     *
     * @param module     (ID)
     * @param moduleType (CTREPCM, REVPH)
     */
    public CompressorImpl(int module, PneumaticsModuleType moduleType) {
        super(module, moduleType);
    }

    /**
     * Alternatively instantiates a CompressorImpl by inferring a default ID of 0 if CTREPCM and 1 if REV
     *
     * @param moduleType (CTREPCM, REVPH)
     */
    public CompressorImpl(PneumaticsModuleType moduleType) {
        super(moduleType);
    }

    /**
     * @see Compressor#getCurrent()
     */
    @Override
    public double getCompressorCurrent() {
        return super.getCurrent();
    }

    /**
     * @see Compressor#getPressureSwitchValue()
     */
    @Override
    public boolean getCompressorSwitchValue() {
        return super.getPressureSwitchValue();
    }
}
