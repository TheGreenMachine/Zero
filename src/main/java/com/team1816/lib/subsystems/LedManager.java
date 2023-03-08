package com.team1816.lib.subsystems;

import com.google.inject.Inject;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.ledManager.ILEDManager;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

import javax.inject.Singleton;
import java.awt.*;

/**
 * Subsystem container for an LEDManager
 *
 * @see ILEDManager
 */
@Singleton
public class LedManager extends Subsystem {

    /**
     * Properties
     */
    public static final String NAME = "ledmanager";

    private static final boolean RAVE_ENABLED =
        factory.getConstant(NAME, "raveEnabled") > 0;
    private static final double RAVE_SPEED = factory.getConstant(NAME, "raveSpeed", 1.0);
    private static final int MAX = (int) factory.getConstant(NAME, "maxLevel", 255);
    private static final int LED_STRIP_COUNT = (int) factory.getConstant(NAME, "ledStripCount", 0);

    /**
     * Components
     */
    private final ILEDManager ledManager;

    /**
     * State
     */
    private boolean blinkLedOn = false;
    private boolean outputsChanged = false;

    private int ledR;
    private int ledG;
    private int ledB;

    private int period; // ms
    private long lastWriteTime = System.currentTimeMillis();
    private LedControlState controlState = LedControlState.SOLID;
    private RobotStatus defaultStatus = RobotStatus.DISABLED;
    private float raveHue;
    private Color lastRaveColor;

    /**
     * Base enum for LED states
     */
    public enum LedControlState {
        RAVE,
        BLINK,
        SOLID,
    }

    /**
     * Instantiates an LedManager with base subsystem properties
     *
     * @param inf Infrastructure
     * @param rs  RobotState
     */
    @Inject
    public LedManager(Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        ledManager = factory.getLEDManager(NAME);

        ledR = 0;
        ledG = 0;
        ledB = 0;

    }

    /** Actions */

    /**
     * Sets led color
     *
     * @param r Red
     * @param g Green
     * @param b Bluee
     */
    private void setLedColor(int r, int g, int b) {
        if (ledR != r || ledG != g || ledB != b) {
            ledR = (int) (r / 255.0 * MAX);
            ledG = (int) (g / 255.0 * MAX);
            ledB = (int) (b / 255.0 * MAX);
            outputsChanged = true;
        }
    }

    /**
     * Indicates status
     *
     * @param status RobotStatues
     * @see RobotStatus
     */
    public void indicateStatus(RobotStatus status) {
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void indicateStatus(RobotStatus status, LedControlState controlState) {
        setLedControlState(controlState);
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void indicateDefaultStatus() {
        indicateStatus(defaultStatus, LedControlState.SOLID);
    }

    public void setLedControlState(LedControlState ledControlState){
        if(ledControlState != controlState){
            this.controlState = ledControlState;
            outputsChanged = true;
        }
    }

    public void setDefaultStatus(RobotStatus defaultStatus) {
        this.defaultStatus = defaultStatus;
        indicateDefaultStatus();
    }

    public double getPeriod() {
        return period;
    }

    private void writeToLed(int r, int g, int b) {
        ledManager.setLEDs(r, g, b, 0, 0, LED_STRIP_COUNT + 8); // 8 == number of camera leds
    }

    /**
     * periodic
     */
    @Override
    public void readFromHardware() {
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            System.out.println(controlState);
            outputsChanged = false;
            switch (controlState) {
                case RAVE:
                    if(RAVE_ENABLED){
                        var color = Color.getHSBColor(raveHue, 1.0f, MAX / 255.0f);
                        if (!color.equals(lastRaveColor)) {
                            outputsChanged = true;
                            writeToLed(color.getRed(), color.getGreen(), color.getBlue());
                        }
                        raveHue += RAVE_SPEED;
                    }
                    break;
                case BLINK:
                    if (System.currentTimeMillis() >= lastWriteTime + (period / 2)) {
                        if (blinkLedOn) {
                            outputsChanged = true;
                            writeToLed(0, 0, 0);
                            blinkLedOn = false;
                        } else {
                            outputsChanged = true;
                            writeToLed(ledR, ledG, ledB);
                            blinkLedOn = true;
                        }
                        lastWriteTime = System.currentTimeMillis();
                    }
                    break;
                case SOLID:
                    writeToLed(ledR, ledG, ledB);
                    break;
            }
        }
    }

    /** Config and Tests */

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {
    }

    /**
     * Tests the subsystem
     *
     * @return returns true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        // no checking performed
        System.out.println("Checking LED systems");
        controlState = LedControlState.SOLID;
        setLedColor(MAX, 0, 0); // set red
        testDelay();
        setLedColor(0, MAX, 0); // set green
        testDelay();
        setLedColor(0, 0, MAX); // set blue
        testDelay();
        return true;
    }

    /**
     * Tests set latency
     */
    private void testDelay() {
        writeToHardware();
        Timer.delay(1.5);
    }

    /**
     * Initializes SendableBuilder for SmartDashboard
     *
     * @param builder SendableBuilder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
    }

    /**
     * Base enum for RobotStatus
     */
    public enum RobotStatus {
        ENABLED(0, MAX, 0), // green
        DISABLED(MAX, MAX / 4, 0), // orange
        ERROR(MAX, 0, 0), // red
        AUTONOMOUS(0, MAX, MAX), // cyan
        ENDGAME(0, 0, MAX), // blue
        CUBE(MAX, 0, 150), // magenta
        RAGE(MAX, 5, 5), // deep magenta
        CONE(MAX, 50, 0), // yellow,
        ON_TARGET(MAX, MAX, MAX), // white
        BALANCE(50, 50, MAX), // light blue
        OFF(0, 0, 0); // off

        final int red;
        final int green;
        final int blue;

        RobotStatus(int r, int g, int b) {
            this.red = r;
            this.green = g;
            this.blue = b;
        }

        public int getRed() {
            return red;
        }

        public int getGreen() {
            return green;
        }

        public int getBlue() {
            return blue;
        }
    }
}
