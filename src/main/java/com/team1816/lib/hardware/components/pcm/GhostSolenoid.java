package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * This class emulates the behaviour of a Solenoid that is not physically implemented on a robot
 */
public class GhostSolenoid implements ISolenoid {

    // State
    private boolean on;

    /**
     * Returns the state of the solenoid
     *
     * @return on
     * @see ISolenoid#get()
     */
    @Override
    public boolean get() {
        return on;
    }

    /**
     * Sets the state of the solenoid
     *
     * @param on (state)
     * @see ISolenoid#set(boolean)
     */
    @Override
    public void set(boolean on) {
        this.on = on;
    }

    /**
     * Alternately toggles between states
     *
     * @see ISolenoid#toggle()
     */
    @Override
    public void toggle() {
        set(!get());
    }

    /**
     * @param builder
     * @see ISolenoid#initSendable(SendableBuilder)
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Solenoid");
        builder.setActuator(true);
        builder.setSafeState(() -> set(false));
        builder.addBooleanProperty("Value", this::get, this::set);
    }

    @Override
    public void close() throws Exception {
    }
}
