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

    private static final int LED_COUNT = (int) factory.getConstant(NAME, "ledCount");
    private static final boolean RAVE_ENABLED =
        factory.getConstant(NAME, "raveEnabled") > 0;
    private static final double RAVE_SPEED = factory.getConstant(NAME, "raveSpeed", 1.0);
    private static final int MAX = (int) factory.getConstant(NAME, "maxLevel", 255);


    /**
     * Components
     */
    private final ILEDManager ledManager;

    /**
     * State
     */
    private boolean outputsChanged = false;
    private int period; // ms
    private long lastWriteTime = System.currentTimeMillis();
    private LedControlState controlState = LedControlState.STANDARD;
    private RobotStatus defaultStatus = RobotStatus.DISABLED;


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

    /**
     * Sets led color
     *
     * @param r Red
     * @param g Green
     * @param b Bluee
     */
    private void setLedColor(int r, int g, int b) {
        if (ledR != r || ledG != g || ledB != b) {
            ledR = r;
            ledG = g;
            ledB = b;
            outputsChanged = true;
        }
    }

    /**
     * Sets LEDs to blink with a certain color
     *
     * @param r LED color red value (0-255)
     * @param g LED color green value (0-255)
     * @param b LED color blue value (0-255)
     */
    private void setLedColorBlink(int r, int g, int b) {
        setLedColor(r, g, b);
        controlState = LedControlState.BLINK;
        this.period = 1000;
        outputsChanged = true;
    }

    /**
     * Indicates status
     *
     * @param status RobotStatues
     * @see RobotStatus
     */
    public void indicateStatus(RobotStatus status) {
        controlState = LedControlState.STANDARD;
        setLedColor(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void indicateDefaultStatus() {
        if (RAVE_ENABLED && defaultStatus != RobotStatus.DISABLED) {
            controlState = LedControlState.RAVE;
            setLedColor(0, 0, 0);
        } else {
            indicateStatus(defaultStatus);
        }
    }

    public void blinkStatus(RobotStatus status) {
        setLedColorBlink(status.getRed(), status.getGreen(), status.getBlue());
    }

    public void setDefaultStatus(RobotStatus defaultStatus) {
        this.defaultStatus = defaultStatus;
        indicateDefaultStatus();
    }

    public double getPeriod() {
        return period;
    }

    public void writeToLed(int r, int g, int b) {
        ledManager.setLEDs(255, 255, 255, 0, 0, 8); // CANdle LEDs
        ledManager.setLEDs(r, g, b, 0, 0, 10 - 8);
    }

    /**
     * Periodic
     */
    @Override
    public void readFromHardware() {
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (controlState) {
                case RAVE:
                    var color = Color.getHSBColor(raveHue, 1.0f, MAX / 255.0f);
                    if (!color.equals(lastRaveColor)) {
                        outputsChanged = true;
                        writeToLed(color.getRed(), color.getGreen(), color.getBlue());
                    }
                    raveHue += RAVE_SPEED;
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
                case STANDARD:
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
        controlState = LedControlState.SINGLE_COLOR_SOLID;
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
     * Base class for Colors
     */
    public static class LEDColor {
        /**
         * Primary
         */
        public static Color strontium = new Color(1*MAX, 0*MAX, 0*MAX); // red
        public static Color boron = new Color(0*MAX, 1*MAX, 0*MAX); // green
        public static Color selenium = new Color(0*MAX, 0*MAX, 1*MAX); // blue

        /**
         * Secondary
         */
        public static Color sodium = new Color(1*MAX, 1*MAX, 0*MAX); // yellow
        public static Color copper = new Color(0*MAX, 1*MAX, 1*MAX); // cyan
        public static Color potassium = new Color(1*MAX, 0*MAX, 1*MAX); // magenta
        public static Color calcium = new Color(1*MAX, (int) (1d/5 * MAX), 0*MAX); // orange

        /**
         * Tertiary
         */
        public static Color arsenic = new Color((int) (155d/255 * MAX), (int) (220d/255 * MAX), (int) (225d/255 * MAX)); // gray
        public static Color magnesium = new Color(1*MAX, 1*MAX, 1*MAX); // white

    }

    /**
     * Base enum for LED states
     */
    public enum LedControlState {
        SINGLE_COLOR_SOLID,
        SINGLE_COLOR_BLINK,
        SINGLE_COLOR_ANIMATION, // CANdle ONLY
        MULTI_COLOR_ANIMATION // CANdle ONLY
    }

    /**
     * Base class for an LEDSegment
     */
    private static class LEDSegment {
        public int length;
        public Color color;
        public LedControlState ledControlState;

        public LEDSegment() {
            length = 0;
            color = Color.BLACK;
            ledControlState = LedControlState.SINGLE_COLOR_SOLID;
        }

        public LEDSegment(int l, Color c) {
            length = l;
            color = c;
            ledControlState = LedControlState.SINGLE_COLOR_SOLID;
        }

        public LEDSegment(int l, Color c, LedControlState lcs) {
            length = l;
            color = c;
            ledControlState = lcs;
        }
    }


    /**
     * Base enum for RobotStatus
     * TODO: REWRITE with LED_SEGMENT
     */
    public enum RobotStatus {
        ENABLED(0, MAX, 0), // green
        DISABLED(MAX, MAX / 5, 0), // orange
        ERROR(MAX, 0, 0), // red
        AUTONOMOUS(0, MAX, MAX), // cyan
        ENDGAME(0, 0, MAX), // blue
        SEEN_TARGET(MAX, 0, MAX), // magenta
        ON_TARGET(MAX, 0, 20), // deep magenta
        DRIVETRAIN_FLIPPED(MAX, MAX, 0); // yellow,


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
