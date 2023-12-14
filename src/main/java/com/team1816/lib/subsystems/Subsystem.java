package com.team1816.lib.subsystems;

import com.google.inject.Inject;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.ILooper;
import com.team1816.season.Robot;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly.
 * <p>
 * Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * initializing all member components on match start.
 */
public abstract class Subsystem implements Sendable {

    /**
     * Properties
     */
    private final String name;
    public static RobotFactory factory = Injector.get(RobotFactory.class);

    public static RobotState robotState;

    public static Infrastructure infrastructure;


    /**
     * Logging
     */
    protected DataLogEntry desStatesLogger;
    protected DataLogEntry actStatesLogger;

    /**
     * Base parameters needed to instantiate a subsystem
     *
     * @param name String
     * @param inf  Infrastructure
     * @param rs   RobotState
     */
    @Inject
    public Subsystem(String name, Infrastructure inf, RobotState rs) {
        this.name = name;
        robotState = rs;
        infrastructure = inf;
    }

    /** Read/Write Periodic */

    /**
     * Reads outputs written by hardware i.e. sensors and similar
     */
    public abstract void readFromHardware(); // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.

    /**
     * Writes outputs to hardware i.e. motors and solenoids
     */
    public abstract void writeToHardware(); // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN

    /** Tests, logging, and general functionality */

    /**
     * Registers all subsystems on their own loops, through the ILooper
     *
     * @param mEnabledLooper ILooper
     * @see ILooper
     */
    public void registerEnabledLoops(ILooper mEnabledLooper) {
    }

    /**
     * Zeroes the subsystem and its sensors to a known zero state for position control
     */
    public abstract void zeroSensors();

    /**
     * Stops the subsystem
     */
    public abstract void stop();

    /**
     * Tests the subsystem based on various criteria
     *
     * @return true if tests passed
     */
    public abstract boolean testSubsystem();

    /**
     * Initializes a SmartDashboard / ShuffleBoard SendableBuilder to convey subsystem information
     *
     * @param builder SendableBuilder
     * @see SendableBuilder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
    }

    /**
     * Returns the subsystem name
     *
     * @return name
     */
    public String getSubsystemName() {
        return name;
    }

    /**
     * Returns whether the subsystem is implemented or is in ghost mode
     *
     * @return boolean implemented
     */
    public boolean isImplemented() {
        return factory.getSubsystem(name).implemented;
    }
}
