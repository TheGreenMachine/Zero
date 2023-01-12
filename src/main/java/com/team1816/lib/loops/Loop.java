package com.team1816.lib.loops;

/**
 * Base interface for loops. Loops are routine that run periodically in the robot code (such as subsystem actions and responses).
 *
 * @see com.team1816.lib.subsystems.Subsystem
 * @see com.team1816.lib.subsystems.SubsystemLooper
 */
public interface Loop {
    /**
     * Action to commence when the loop is started
     *
     * @see com.team1816.lib.subsystems.SubsystemLooper {EnabledLoop#onStart}
     * \     * @param timestamp
     */
    void onStart(double timestamp);

    /**
     * Periodic action to occur when the loop is running
     *
     * @param timestamp
     * @see com.team1816.lib.subsystems.SubsystemLooper {EnabledLoop#onLoop}
     */
    void onLoop(double timestamp);

    /**
     * Periodic action to occur when the loop will stop
     *
     * @param timestamp
     * @see com.team1816.lib.subsystems.SubsystemLooper {EnabledLoop#onStop}
     */
    void onStop(double timestamp);
}
