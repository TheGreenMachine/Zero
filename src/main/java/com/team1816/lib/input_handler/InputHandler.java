package com.team1816.lib.input_handler;

import com.team1816.lib.events.EventAggregator;
import com.team1816.lib.events.PubSubConsumer;
import com.team1816.lib.events.PubSubRunnable;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.team254.LatchedBoolean;
import edu.wpi.first.wpilibj.Joystick;
import org.checkerframework.checker.units.qual.A;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.function.Consumer;

/**
 * Handles all the input coming from the main three controllers
 */
@Singleton
public class InputHandler {
    protected class ButtonData {
        public Integer buttonId = -1;
        public ArrayList<Runnable> pressActions = new ArrayList<>();
        public ArrayList<Runnable> releaseActions = new ArrayList<>();
        public ArrayList<Runnable> holdActions = new ArrayList<>();
        public final LatchedBoolean pressedState = new LatchedBoolean();
        public final LatchedBoolean releasedState = new LatchedBoolean();

        public void publish(boolean held) {
            boolean justPressed = pressedState.update(held);
            boolean justReleased = releasedState.update(!held);

            if (justPressed) {
                for (var action : pressActions) {
                    action.run();
                }
            }

            if (held) {
                for (var action : holdActions) {
                    action.run();
                }
            }

            if (justReleased) {
                for (var action : releaseActions) {
                    action.run();
                }
            }
        }
    }

    protected class AxisData {
        public Integer axisId = -1;
        public ArrayList<Consumer<Double>> actions = new ArrayList<>();

        public void publish(double value) {
            for (var action : actions) {
                action.accept(value);
            }
        }
    }

    private EnumMap<Button, ButtonData> driverButtonMappings = new EnumMap<>(Button.class);
    private EnumMap<Button, ButtonData> operatorButtonMappings = new EnumMap<>(Button.class);
    private EnumMap<Button, ButtonData> buttonBoardButtonMappings = new EnumMap<>(Button.class);

    private EnumMap<Axis, AxisData> driverAxisMappings = new EnumMap<>(Axis.class);
    private EnumMap<Axis, AxisData> operatorAxisMappings = new EnumMap<>(Axis.class);
    private EnumMap<Axis, AxisData> buttonBoardAxisMappings = new EnumMap<>(Axis.class);

    private Joystick driverJoystick;
    private Joystick operatorJoystick;
    private Joystick buttonBoardJoystick;

    private ControllerBinding driverBinding;
    private ControllerBinding operatorBinding;
    private ControllerBinding buttonBoardBinding;

    private final EventAggregator eventAggregator;

    public final int driverPort = 0;
    private final int operatorPort = 1;
    private final int buttonBoardPort = 2;

    public void listenDriverButton(Button button, Button.State state, Runnable action) {
        if (!driverButtonMappings.containsKey(button)) {
            GreenLogger.log("   INPUT HANDLER: BUTTON " + button.toString() + " is not mapped.");
            return;
        }

        ButtonData data = driverButtonMappings.get(button);

        switch (state) {
            case HOLD -> data.holdActions.add(action);
            case PRESSED ->  data.pressActions.add(action);
            case RELEASED -> data.releaseActions.add(action);
        }
    }

    public void listenDriverAxis(Axis axis, Consumer<Double> action) {
        if (!driverAxisMappings.containsKey(axis)) {
            GreenLogger.log("   INPUT HANDLER: AXIS " + axis.toString() + " is not mapped.");
            return;
        }

        AxisData data = driverAxisMappings.get(axis);

        data.actions.add(action);
    }

    public void listenOperatorButton(Button button, Button.State state, Runnable action) {
        if (!operatorButtonMappings.containsKey(button)) {
            GreenLogger.log("   INPUT HANDLER: BUTTON " + button.toString() + " is not mapped.");
            return;
        }

        ButtonData data = operatorButtonMappings.get(button);

        switch (state) {
            case HOLD -> data.holdActions.add(action);
            case PRESSED ->  data.pressActions.add(action);
            case RELEASED -> data.releaseActions.add(action);
        }
    }

    public void listenOperatorAxis(Axis axis, Consumer<Double> action) {
        if (!operatorAxisMappings.containsKey(axis)) {
            GreenLogger.log("   INPUT HANDLER: AXIS " + axis.toString() + " is not mapped.");
            return;
        }

        AxisData data = operatorAxisMappings.get(axis);

        data.actions.add(action);
    }

    public void listenButtonBoardButton(Button button, Button.State state, Runnable action) {
        if (!buttonBoardButtonMappings.containsKey(button)) {
            GreenLogger.log("   INPUT HANDLER: BUTTON " + button.toString() + " is not mapped.");
            return;
        }

        ButtonData data = buttonBoardButtonMappings.get(button);

        switch (state) {
            case HOLD -> data.holdActions.add(action);
            case PRESSED ->  data.pressActions.add(action);
            case RELEASED -> data.releaseActions.add(action);
        }
    }

    public void listenButtonBoardAxis(Axis axis, Consumer<Double> action) {
        if (!buttonBoardAxisMappings.containsKey(axis)) {
            GreenLogger.log("   INPUT HANDLER: AXIS " + axis.toString() + " is not mapped.");
            return;
        }

        AxisData data = buttonBoardAxisMappings.get(axis);

        data.actions.add(action);
    }

    public void update() {
        // Updating buttons
        for (var button : driverButtonMappings.keySet()) {
            ButtonData driverButtonData = driverButtonMappings.get(button);

            boolean held = driverJoystick.getRawButton(driverButtonData.buttonId);

            driverButtonData.publish(held);
        }

        for (var button : operatorButtonMappings.keySet()) {
            ButtonData operatorButtonData = operatorButtonMappings.get(button);

            boolean held = operatorJoystick.getRawButton(operatorButtonData.buttonId);

            operatorButtonData.publish(held);
        }

        for (var button : buttonBoardButtonMappings.keySet()) {
            ButtonData buttonBoardButtonData = buttonBoardButtonMappings.get(button);

            boolean held = buttonBoardJoystick.getRawButton(buttonBoardButtonData.buttonId);

            buttonBoardButtonData.publish(held);
        }

        // Updating Axes
        for (var axis : driverAxisMappings.keySet()) {
            AxisData driverAxisData = driverAxisMappings.get(axis);

            double value = driverJoystick.getRawAxis(driverAxisData.axisId);

            driverAxisData.publish(value);
        }

        for (var axis : operatorAxisMappings.keySet()) {
            AxisData operatorAxisData = operatorAxisMappings.get(axis);

            double value = operatorJoystick.getRawAxis(operatorAxisData.axisId);

            operatorAxisData.publish(value);
        }

        for (var axis : buttonBoardAxisMappings.keySet()) {
            AxisData buttonBoardAxisData = buttonBoardAxisMappings.get(axis);

            double value = buttonBoardJoystick.getRawAxis(buttonBoardAxisData.axisId);

            buttonBoardAxisData.publish(value);
        }
    }

    @Inject
    public InputHandler() {
        eventAggregator = new EventAggregator();

        // NOTE(Michael): This is a temporary thing to test whether the mappings work.
        driverBinding = new XboxControllerBinding();
        operatorBinding = new WasdControllerBinding();
        buttonBoardBinding = new ButtonBoardControllerBinding();

        driverJoystick = new Joystick(driverPort);
        operatorJoystick = new Joystick(operatorPort);
        buttonBoardJoystick = new Joystick(buttonBoardPort);
    }

    public void init() {
        // Filling Button Mappings
        for (var button : driverBinding.buttonMap.keySet()) {
            Integer driverButtonId = driverBinding.buttonMap.get(button);
            ButtonData driverButtonData = new ButtonData();

            driverButtonData.buttonId = driverButtonId;

            driverButtonMappings.put(button, driverButtonData);
        }

        for (var button : operatorBinding.buttonMap.keySet()) {
            Integer operatorButtonId = operatorBinding.buttonMap.get(button);
            ButtonData operatorButtonData = new ButtonData();

            operatorButtonData.buttonId = operatorButtonId;

            operatorButtonMappings.put(button, operatorButtonData);
        }

        for (var button : buttonBoardBinding.buttonMap.keySet()) {
            Integer buttonBoardButtonId = buttonBoardBinding.buttonMap.get(button);
            ButtonData buttonBoardButtonData = new ButtonData();

            buttonBoardButtonData.buttonId = buttonBoardButtonId;

            buttonBoardButtonMappings.put(button, buttonBoardButtonData);
        }

        // Filling Axis Mappings
        for (var axis : driverBinding.axisMap.keySet()) {
            Integer driverAxisId = driverBinding.axisMap.get(axis);
            AxisData driverAxisData = new AxisData();

            driverAxisData.axisId = driverAxisId;

            driverAxisMappings.put(axis, driverAxisData);
        }

        for (var axis : operatorBinding.axisMap.keySet()) {
            Integer operatorAxisId = operatorBinding.axisMap.get(axis);
            AxisData operatorAxisData = new AxisData();

            operatorAxisData.axisId = operatorAxisId;

            operatorAxisMappings.put(axis, operatorAxisData);
        }

        for (var axis : buttonBoardBinding.axisMap.keySet()) {
            Integer buttonBoardAxisId = buttonBoardBinding.axisMap.get(axis);
            AxisData buttonBoardAxisData = new AxisData();

            buttonBoardAxisData.axisId = buttonBoardAxisId;

            buttonBoardAxisMappings.put(axis, buttonBoardAxisData);
        }
    }
}
