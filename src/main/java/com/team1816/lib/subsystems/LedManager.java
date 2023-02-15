package com.team1816.lib.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.google.inject.Inject;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.ledManager.CANdleImpl;
import com.team1816.lib.hardware.components.ledManager.CanifierImpl;
import com.team1816.lib.hardware.components.ledManager.ILEDManager;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.season.states.RobotState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

import javax.inject.Singleton;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;

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

    private static final int LED_STRIP_COUNT = (int) factory.getConstant(NAME, "ledStripCount", 0);
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
    private RobotStatus defaultStatus = RobotStatus.DISABLED;
    private RobotStatus controlStatus;
    private List<LEDSegment> segments;

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

        controlStatus = RobotStatus.DISABLED;
        segments = controlStatus.getSegments();
    }

    /**
     * Sets control status
     * @param status robot status
     */
    private void setControlStatus(RobotStatus status) {
        if (controlStatus != status) {
            controlStatus = status;
            segments = status.getSegments();
            outputsChanged = true;
        }
    }

    /**
     * Sets the default status
     * @param defaultStatus default robot status
     */
    public void setDefaultStatus(RobotStatus defaultStatus) {
        this.defaultStatus = defaultStatus;
        indicateDefaultStatus();
    }

    /**
     * Gets control status
     */
    public RobotStatus getStatus() {
        return controlStatus;
    }

    /**
     * Sets led color
     *
     * @param r Red
     * @param g Green
     * @param b Blue
     */
    private void setLedColor(int r, int g, int b) {
        writeToLed(r, g, b);
        outputsChanged = true;
    }

    /**
     * Sets led color
     *
     * @param r Red
     * @param g Green
     * @param b Blue
     */
    private void setLedColor(int r, int g, int b, int startIdx, int count) {
        writeToLed(r, g, b, startIdx, count);
        outputsChanged = true;
    }

    /**
     * Indicates status
     *
     * @param status RobotStatues
     * @see RobotStatus
     */
    public void indicateStatus(RobotStatus status) {
        setControlStatus(status);
    }

    /**
     * Indicates the default robot status
     */
    public void indicateDefaultStatus() {
        indicateStatus(defaultStatus);
    }

    /**
     * Returns the blinking periodicity
     * @return period
     */
    public double getPeriod() {
        return period;
    }

    /**
     * TODO RENAME
     * Writes a specific color to the LEDs
     *
     * @param r red value [0, 255]
     * @param g green value [0, 255]
     * @param b blue value [0, 255]
     */
    public void writeToLed(int r, int g, int b, int startIdx, int count) {
        ledManager.setLEDs(255, 255, 255, 0, 0, 8); // CANdle LEDs
        ledManager.setLEDs(r, g, b, 0, startIdx, count);
        outputsChanged = false;
    }

    /**
     * TODO RENAME
     * Writes a specific color to the LEDs
     *
     * @param r red value [0, 255]
     * @param g green value [0, 255]
     * @param b blue value [0, 255]
     */
    public void writeToLed(int r, int g, int b) {
        ledManager.setLEDs(255, 255, 255, 0, 0, 8); // CANdle LEDs
        ledManager.setLEDs(r, g, b, 0, 8, LED_STRIP_COUNT);
        outputsChanged = false;
    }

    /** TODO ADD METHOD HERE FOR FIXED SEGMENTS */

    /**
     * Writes led segments to the LEDs periodically based on segment control states
     *
     * @param segments LEDSegments
     */
    public void writeToLed(List<LEDSegment> segments) {
        int count = 0;
        for (int i = 0; i < segments.size(); i++) {
            switch (segments.get(i).ledControlState) {
                case SINGLE_COLOR_SOLID -> {
                    var segmentColor = segments.get(i).color;
                    setLedColor(segmentColor.getRed(), segmentColor.getGreen(), segmentColor.getBlue(), count, segments.get(i).length);
                } case SINGLE_COLOR_BLINK -> {
                    var segmentColor = segments.get(i).color;
                    setLedColor(segmentColor.getRed(), segmentColor.getGreen(), segmentColor.getBlue(), count, segments.get(i).length);
                    if (System.currentTimeMillis() >= lastWriteTime + (period / 2)) {
                        if (segments.get(i).blinkLedOn) {
                            outputsChanged = true;
                            setLedColor(0, 0, 0, count, segments.get(i).length);
                            segments.get(i).blinkLedOn = false;
                        } else {
                            outputsChanged = true;
                            setLedColor(segmentColor.getRed(), segmentColor.getGreen(), segmentColor.getBlue(), count, segments.get(i).length);
                            segments.get(i).blinkLedOn = true;
                        }
                        lastWriteTime = System.currentTimeMillis();
                    }
                } case SINGLE_COLOR_ANIMATION -> {
                    var segmentColor = segments.get(i).color;
                    if (segments.get(i).animation == null) {
                        ((CANdle) ledManager).animate(
                            new StrobeAnimation(
                                segmentColor.getRed(), segmentColor.getGreen(), segmentColor.getBlue(), 0, 0.25, count, segments.get(i).length
                            )
                        );
                    } else {
                        ((CANdle) ledManager).animate(
                            segments.get(i).animation
                        );
                    }
                } case MULTI_COLOR_ANIMATION -> {
                    if (segments.get(i).animation == null) {
                        ((CANdle) ledManager).animate(
                            new RainbowAnimation()
                        );
                    } else {
                        ((CANdle) ledManager).animate(
                            segments.get(i).animation
                        );
                    }
                }
            }
            count += segments.get(i).length;
        }
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
            if (ledManager instanceof CanifierImpl) {
                segments = controlStatus.getSegments();

                // reformat segments
                if (segments.size() > 0) {
                    var seg = segments.get(0);
                    if (seg.ledControlState == LedControlState.SINGLE_COLOR_ANIMATION || seg.ledControlState == LedControlState.MULTI_COLOR_ANIMATION) {
                        segments.set(0, new LEDSegment(LED_STRIP_COUNT, seg.color, LedControlState.SINGLE_COLOR_BLINK));
                    } else {
                        segments.set(0, new LEDSegment(LED_STRIP_COUNT, seg.color, seg.ledControlState));
                    }
                    segments = new ArrayList<>(List.of(segments.get(0)));
                }
                // TODO write outputs
                writeToLed(segments);
            }
            if (ledManager instanceof CANdleImpl) {
                int count = 0;
                segments = controlStatus.getSegments();

                // reformat segments
                for (int i = 0; i < segments.size(); i++) {
                    var seg = segments.get(i);
                    if (count > LED_STRIP_COUNT) {
                        segments.set(i, new LEDSegment(0, seg.color, seg.ledControlState));
                    }
                    if (seg.length + count > LED_STRIP_COUNT) {
                        segments.set(i, new LEDSegment(Math.max(0, LED_STRIP_COUNT - count), seg.color, seg.ledControlState));
                    }
                }
                // TODO write outputs
                writeToLed(segments);
            }
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(
            new Loop() {
                double startTime;
                @Override
                public void onStart(double timestamp) {
                    startTime = timestamp;
                }

                @Override
                public void onLoop(double timestamp) {
                    writeToHardware();
                }

                @Override
                public void onStop(double timestamp) {

                }
            }
        );
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
        Timer.delay(1);
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
        public static Color STRONTIUM = new Color(1*MAX, 0*MAX, 0*MAX); // red
        public static Color BORON = new Color(0*MAX, 1*MAX, 0*MAX); // green
        public static Color INDIUM = new Color(0*MAX, 0*MAX, 1*MAX); // blue

        /**
         * Secondary
         */
        public static Color SODIUM = new Color(1*MAX, 1*MAX, 0*MAX); // yellow
        public static Color COPPER = new Color(0*MAX, 1*MAX, 1*MAX); // cyan
        public static Color POTASSIUM = new Color(1*MAX, 0*MAX, 1*MAX); // magenta
        public static Color CALCIUM = new Color(1*MAX, (int) (1d/5 * MAX), 0*MAX); // orange

        /**
         * Tertiary
         */
        public static Color ARSENIC = new Color((int) (155d/255 * MAX), (int) (220d/255 * MAX), (int) (225d/255 * MAX)); // gray
        public static Color MAGNESIUM = new Color(1*MAX, 1*MAX, 1*MAX); // white

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
        public Animation animation;
        public boolean blinkLedOn = false;

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

        public LEDSegment(int l, Color c, LedControlState lcs, Animation a) {
            length = l;
            color = c;
            ledControlState = lcs;
            animation = a;
        }
    }


    /**
     * Base enum for RobotStatus
     */
    public enum RobotStatus {
        ENABLED(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.BORON, LedControlState.SINGLE_COLOR_SOLID)))), // solid green
        DISABLED(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.CALCIUM, LedControlState.SINGLE_COLOR_SOLID)))), // solid orange
        ERROR(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.STRONTIUM, LedControlState.SINGLE_COLOR_SOLID)))), // solid red

        AUTONOMOUS(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.COPPER, LedControlState.SINGLE_COLOR_SOLID)))), // solid cyan
        ENDGAME(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.INDIUM, LedControlState.SINGLE_COLOR_SOLID)))), // solid blue

        CONE(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.SODIUM, LedControlState.SINGLE_COLOR_SOLID)))), // solid yellow
        CUBE(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.POTASSIUM, LedControlState.SINGLE_COLOR_SOLID)))), // solid purple
        CONTAINS_OBJECT(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.MAGNESIUM, LedControlState.SINGLE_COLOR_SOLID)))), // solid white
        COLLECT(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.STRONTIUM, LedControlState.SINGLE_COLOR_BLINK)))), // blinking red
        SEEN_TARGET(new ArrayList<>(List.of(new LEDSegment(LED_STRIP_COUNT, LEDColor.ARSENIC, LedControlState.SINGLE_COLOR_BLINK)))); // blinking light gray


        private ArrayList<LEDSegment> segments;
        RobotStatus(ArrayList<LEDSegment> segments) {
            this.segments = segments;
        }

        public List<LEDSegment> getSegments() {
            return segments;
        }

        public void setSegments(ArrayList<LEDSegment> segments) {
            this.segments = segments;
        }
    }
}
