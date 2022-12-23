package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * This class emulates the behaviour of a Compressor that is not physically implemented on a robot
 * @see ICompressor
 */
public class GhostCompressor implements ICompressor {

    private boolean enabled = false;

    @Override
    public void enableDigital() {
        this.enabled = true;
    }

    @Override
    public void disable() {
        this.enabled = false;
    }

    @Override
    public boolean enabled() {
        return enabled;
    }

    @Override
    public double getCompressorCurrent() {
        return 0;
    }

    @Override
    public boolean getCompressorSwitchValue() {
        return false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Compressor");
        builder.addBooleanProperty(
            "Enabled",
            this::enabled,
            value -> {
                if (value) {
                    enableDigital();
                } else {
                    disable();
                }
            }
        );
    }

    @Override
    public void close() throws Exception {}
}
