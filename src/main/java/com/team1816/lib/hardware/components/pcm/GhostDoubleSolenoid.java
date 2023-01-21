package com.team1816.lib.hardware.components.pcm;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * This class emulates the behaviour of a DoubleSolenoid that is not physically implemented on a robot
 */
public class GhostDoubleSolenoid implements IDoubleSolenoid {

    // State
    private Value value;

    /**
     * Returns the value of the double solenoid
     *
     * @return value
     */
    @Override
    public Value get() {
        return value;
    }

    /**
     * Sets the value of the double solenoid
     *
     * @param value (Value)
     */
    @Override
    public void set(Value value) {
        this.value = value;
    }

    /**
     * Alternately toggles the solenoid value
     */
    @Override
    public void toggle() {
        if (value == Value.kForward) {
            set(Value.kReverse);
        } else if (value == Value.kReverse) {
            set(Value.kForward);
        }
    }

    /**
     * @param builder
     * @see IDoubleSolenoid#initSendable(SendableBuilder)
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Double Solenoid");
        builder.setActuator(true);
        builder.setSafeState(() -> set(Value.kOff));
        builder.addStringProperty(
            "Value",
            () -> get().name().substring(1),
            value -> {
                if ("Forward".equals(value)) {
                    set(Value.kForward);
                } else if ("Reverse".equals(value)) {
                    set(Value.kReverse);
                } else {
                    set(Value.kOff);
                }
            }
        );
    }

    @Override
    public void close() throws Exception {
    }
}
