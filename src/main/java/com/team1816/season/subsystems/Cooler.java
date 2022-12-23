package com.team1816.season.subsystems;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * This subsystem models a compressed air cooling system for motors to ensure optimal performance and prevent overheating.
 * This is also an example of a subsystem that needs no marked difference between desired and actual states and utilizes
 * a thermostat model of temperature regulation.
 */
@Singleton
public class Cooler extends Subsystem {

    /**
     * Properties
     */
    private static final String NAME = "cooler";

    protected final double heatThreshold = factory.getConstant(
        NAME,
        "heatThreshold",
        100
    );

    /**
     * Components
     */
    private final ISolenoid lock;
    private final ISolenoid dump;

    /**
     * State
     */
    private boolean needsDump = false;
    private boolean outputsChanged = false;
    private boolean shutDown = false;

    private final boolean letAirFlow = true;
    private final boolean blockAirFlow = false;
    private AsyncTimer coolTimer;

    /**
     * Instantiates a collector from the base subsystem components
     * @param inf Infrastructure
     * @param rs RobotState
     */
    @Inject
    public Cooler(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        lock = factory.getSolenoid(NAME, "lock");
        dump = factory.getSolenoid(NAME, "dump");

        coolTimer =
            new AsyncTimer(0.5, () -> lock.set(!needsDump), () -> dump.set(needsDump));
        SmartDashboard.putBoolean("Drive/Overheating", needsDump);
    }

    /**
     * Periodically reads the drivetrain motor temperatures from RobotState and decides if air should be let out
     */
    @Override
    public void readFromHardware() {
        if (robotState.drivetrainTemp > heatThreshold && !needsDump) {
            needsDump = true;
            outputsChanged = true;
        } else if (robotState.drivetrainTemp < heatThreshold - 10 && needsDump) {
            needsDump = false;
            outputsChanged = true;
        }
        if (DriverStation.getMatchTime() > 90) {
            shutDown = true;
            outputsChanged = true;
        }

        if (dump.get() == blockAirFlow) {
            robotState.coolState = STATE.WAIT;
        } else {
            robotState.coolState = STATE.DUMP;
        }
    }

    /**
     * Writes outputs to the solenoids to cool the motors
     */
    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            coolControl();
        }
    }

    /**
     * Main logical operator for cooling the motors
     */
    public void coolControl() {
        if (shutDown) {
            needsDump = false;
            if (!coolTimer.isCompleted()) {
                System.out.println("shutting down cooler");
                coolTimer.update();
                outputsChanged = true;
            }
        } else {
            if (!coolTimer.isCompleted()) {
                coolTimer.update();
                outputsChanged = true;
            } else {
                coolTimer.reset();
                SmartDashboard.putBoolean("Drive/Overheating", needsDump);
            }
        }
    }

    /** Config and Tests */

    /**
     * Resets state variables
     */
    @Override
    public void zeroSensors() {
        needsDump = false;
        outputsChanged = false;
        shutDown = false;
        coolTimer.reset();
    }

    /**
     * Stops the cooler and resets solenoids
     */
    @Override
    public void stop() {
        lock.set(letAirFlow);
        dump.set(blockAirFlow);
    }

    /**
     * Tests the subsystem
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        return true;
    }

    /**
     * Base enum for cooler states
     */
    public enum STATE {
        WAIT,
        DUMP,
    }
}
