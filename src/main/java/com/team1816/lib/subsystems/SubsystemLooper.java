package com.team1816.lib.subsystems;

import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This class establishes an efficient management pattern to run all subsystems on their own loops. As a consequence, it
 * loops through subsystem readFromHardware / writeToHardware methods and provides a framework to enable, update, and
 * disable / stop all subsystems
 *
 * @see Looper
 * @see Loop
 */
public class SubsystemLooper implements ILooper {

    /**
     * Properties
     */
    private List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    /**
     * Instantiates a SubsystemLooper
     */
    public SubsystemLooper() {
    }

    /**
     * Tests all Subsystems
     *
     * @return true if all tests passed
     */
    public boolean testSubsystems() {
        boolean ret_val = true;

        for (Subsystem s : mAllSubsystems) {
            GreenLogger.log("SUBSYSTEM: " + s.getSubsystemName());
            ret_val &= s.testSubsystem();
        }

        return ret_val;
    }

    /**
     * Outputs values to the SmartDashboard / Shuffleboard
     */
    public void outputToSmartDashboard() {
    }

    /**
     * Stops all subsystems
     *
     * @see Subsystem#stop()
     */
    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    /**
     * Zeroes all subsystems
     *
     * @see Subsystem#zeroSensors()
     */
    public void zeroSensors() {
        mAllSubsystems.forEach(Subsystem::zeroSensors);
    }

    /**
     * Returns the list of all registered subsystems
     *
     * @return mAllSubsystems
     */
    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }

    /**
     * Sets the subsystem register based on a variable argument parameter of multiple subsystems
     *
     * @param allSubsystems Subsystem...
     */
    public void setSubsystems(Subsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
        for (Subsystem subsystem : mAllSubsystems) {
            if (!subsystem.isImplemented()) {
                GreenLogger.log(
                    "Warning: " + subsystem.getSubsystemName() + " is not implemented"
                );
            }
        }
        GreenLogger.log("********** Subsystems set **********");
    }

    /**
     * Base class for running an enabled loop or a loop for each subsystem when the robot is enabled
     */
    private class EnabledLoop implements Loop {

        /**
         * Behaviour when loop starts
         *
         * @param timestamp time
         * @see Loop#onStart(double)
         */
        @Override
        public void onStart(double timestamp) {
            mLoops.forEach(l -> l.onStart(timestamp));
        }

        /**
         * Behaviour when loop is iterated / running
         *
         * @param timestamp time
         * @see Loop#onLoop(double)
         */
        @Override
        public void onLoop(double timestamp) {
            // loop through calls assigned by registerEnabledLoops (i.e. in Drive)
            mLoops.forEach(l -> l.onLoop(timestamp));

            // loop through read and write from hardware
            mAllSubsystems.forEach(Subsystem::readFromHardware);
            mAllSubsystems.forEach(Subsystem::writeToHardware);
        }

        /**
         * Behaviour when loop stops
         *
         * @param timestamp time
         * @see Loop#onStop(double)
         */
        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
        }
    }

    /**
     * Base class for running a disabled loop or a loop for each subsystem when the robot is disabled
     */
    private class DisabledLoop implements Loop {

        /**
         * Behaviour when loop starts
         *
         * @param timestamp time
         * @see Loop#onStart(double)
         */
        @Override
        public void onStart(double timestamp) {
        }

        /**
         * Behaviour when loop is iterated / running
         *
         * @param timestamp time
         * @see Loop#onLoop(double)
         */
        @Override
        public void onLoop(double timestamp) {
            // only loop through read from hardware
            mAllSubsystems.forEach(Subsystem::readFromHardware);
        }

        /**
         * Behaviour when loop stops
         *
         * @param timestamp time
         * @see Loop#onStop(double)
         */
        @Override
        public void onStop(double timestamp) {
        }
    }

    /**
     * Registers all enabled loops in the looper
     *
     * @param enabledLooper Looper
     * @see Looper
     */
    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    /**
     * Registers all disabled loops in the looper
     *
     * @param disabledLooper Looper
     * @see Looper
     */
    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    /**
     * Registers a loop
     *
     * @param loop Loop
     * @see Loop
     */
    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}
