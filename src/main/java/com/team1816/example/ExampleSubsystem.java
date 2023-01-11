package com.team1816.example;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;

/** All subsystems have an @Singleton annotation added to the top of their class declaration
 * to make sure that when the Injector is told to inject this subsystem somewhere in the project, it only
 * creates one instance of the subsystem then refers back to said instance in subsequent calls
 * <p>
 * in human lang, this is used to make sure we only create one of each subsystem object
 **/
@Singleton
public class ExampleSubsystem extends Subsystem {

    /**
     * ALL subsystems have the NAME constant
     * <p>string value = the name we give this subsystem in yaml (zero.config.yml)
     */
    // Constants
    private static final String NAME = "examplesubsystem";
    private final double someDouble;
    private final boolean isBrakeMode;

    /**
     * "Component" is the term we use to refer to anything physically on the robot that we're coding
     * - ie Motors, Solenoids (aka pistons), Gyros (aka pigeons), Compressors, Cameras, etc
     * <p>
     *     Most subsystems generally hold some variety of motors and solenoids
     * </p>
     */
    // Components
    private final IGreenMotor launchMotor;
    private final ISolenoid superGigaSuperVacuumPumpingPiston; // plz don't make var names this long - it's painful to look @

    /**
     * A STATE is an enum representing the state of your subsystem - you need to create it yourself
     * (Usually at the bottom of the class)
     * <p>
     * If a subsystem uses some form of feedback control, (ie position or velocity control on your motors), its a good idea to add these in -
     * otherwise it's not necessary
     * </p>
     * Subsystems that use STATE enums to gauge what they're doing are usually controlled by the Orchestrator (Superstructure) along with other state-controlled subsystems
     * <p>
     *     The desiredState refers to the state that the Orchestrator wants this subsystem to be in.
     *     Subsystems should do whatever they can to match their actualState
     *     (what's actually going on with the subsystem on the robot) with the desiredState
     * </p>
     */
    // State
    private STATE desiredState;
    private STATE actualState;
    private double desiredVel;
    private double actualVel;

    /**
     * Constructor for this subsystem:
     * <p>
     * Add the @Inject annotation above the constructor to identify it as something that the Injector needs to create
     * <p>
     * Define subsystem variables here - constants, components, initial states, etc
     * @param inf - passed in by the Injector when this subsystem is first created (by Injector)
     * @param rs - passed in by the Injector when this subsystem is first created (by Injector)
     */
    @Inject
    public ExampleSubsystem(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs); // call to Subsystem constructor to register this subsystem
        /*
            Definition of constants and components is done predominantly by using the factory.
            The factory is responsible for turning what we've put in yaml into defined constants/components
            To get anything from the factory, just call
            factory.get[whatever const/comp you need](NAME - your subsystem in yaml, yaml name of what you need);
         */
        someDouble = factory.getConstant(NAME, "someDouble");
        isBrakeMode = factory.getConstant(NAME, "isBrakeMode") > 0; // since yml constants are doubles by default, convert to bool

        launchMotor = factory.getMotor(NAME, "launchMotor");
        superGigaSuperVacuumPumpingPiston = factory.getSolenoid(NAME, "brad"); // yml name doesn't match class name - try to avoid!

        desiredState = STATE.STOP;
        actualState = STATE.STOP;
    }

    /**
     * readFromHardware is one of two methods that the SubsystemLooper/Manager calls periodically. Within this method,
     * subsystems would update actual component states (motor VelPer100MS, solenoid out or in, etc.),
     * and use them to update subsystem's actual state
     * <p>
     * since readFromHardware isn't intended to tell any components to actually do anything,
     * SubsystemLooper/Manager periodically calls this method in both disabled AND enabled loops
     * </p>
     */
    @Override
    public void readFromHardware() {}

    /**
     * writeToHardware is the second method that the SubsystemLooper/Manager calls periodically. Within this method,
     * subsystems would tell their components to do stuff based on what the Subsystem itself is being told to do
     * (ie: ArmSubsystem that is told to raise its arm would tell its arm motor to engage here based on the subsystem's desired state)
     * <p>
     * since writeToHardware IS intended to tell any components to do stuff,
     * SubsystemLooper/Manager periodically calls this method in enabled loop only
     * </p>
     */
    @Override
    public void writeToHardware() {}

    /**
     * Subsystems use this method to reset component sensors before they're used in a match
     * - this method is usually called in Robot's init methods
     */
    @Override
    public void zeroSensors() {}

    /**
     * Pretty self-explanatory
     */
    @Override
    public void stop() {}

    /**
     * called for all subsystems when robot is running in Test mode
     * @return whether the test passed
     */
    @Override
    public boolean testSubsystem() {
        return false;
    }

    /**
     * STATE enum - holds possible subsystem states - desiredState and actualState are of this type
     */
    public enum STATE {
        STOP,
        SWIM,
        SHUFFLE,
        SLIDE,
        EXPLODE,
    }
}
