package com.team1816.lib.input_handler;

import com.team1816.lib.events.EventAggregator;
import com.team1816.lib.events.PubSubConsumer;
import com.team1816.lib.events.PubSubRunnable;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.team254.LatchedBoolean;
import edu.wpi.first.wpilibj.Joystick;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.EnumMap;

/**
 * Handles all the input coming from the main three controllers
 */
@Singleton
public class InputHandler {
    public static class ButtonPressEvent extends PubSubRunnable {}
    public static class ButtonReleaseEvent extends PubSubRunnable {}
    public static class ButtonHoldEvent extends PubSubRunnable {}
    public static class AxisEvent extends PubSubConsumer<Double> {}

    protected class ButtonData {
        public Integer buttonId;
        public ButtonPressEvent pressEvent;
        public ButtonReleaseEvent releaseEvent;
        public ButtonHoldEvent holdEvent;
        public final LatchedBoolean pressedState = new LatchedBoolean();
        public final LatchedBoolean releasedState = new LatchedBoolean();
    }

    protected class AxisData {
        public Integer axisId;
        public AxisEvent event;
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

    public void listenDriverButtonHold(Button button, Runnable action) {
        if (!driverButtonMappings.containsKey(button)) {
            GreenLogger.log("   INPUT HANDLER: BUTTON " + button.toString() + " is not mapped.");
            return;
        }

        ButtonData data = driverButtonMappings.get(button);

        GreenLogger.log(button.toString());

        data.holdEvent.Subscribe(action);
    }

    public void listenDriverButtonPress(Button button, Runnable action) {
        if (!driverButtonMappings.containsKey(button)) {
            GreenLogger.log("   INPUT HANDLER: BUTTON " + button.toString() + " is not mapped.");
            return;
        }

        ButtonData data = driverButtonMappings.get(button);

        data.pressEvent.Subscribe(action);
    }

    public void listenDriverButtonRelease(Button button, Runnable action) {
        if (!driverButtonMappings.containsKey(button)) {
            GreenLogger.log("   INPUT HANDLER: BUTTON " + button.toString() + " is not mapped.");
            return;
        }

        ButtonData data = driverButtonMappings.get(button);

        data.releaseEvent.Subscribe(action);
    }

    public void update() {
        for (var button : Button.values()) {
            if (!driverButtonMappings.containsKey(button)) {
                continue;
            }

            ButtonData driverButtonData = driverButtonMappings.get(button);

            if (driverButtonData.buttonId != -1) {
                boolean held = driverJoystick.getRawButton(driverButtonData.buttonId);
                boolean justPressed = driverButtonData.pressedState.update(held);
                boolean justReleased = driverButtonData.releasedState.update(!held);

                if (justPressed) driverButtonData.pressEvent.Publish();
                if (held) driverButtonData.holdEvent.Publish();
                if (justReleased) driverButtonData.releaseEvent.Publish();
            }

        }
    }

    @Inject
    public InputHandler() {
        eventAggregator = new EventAggregator();

        // NOTE(Michael): This is a temporary thing to test whether the mappings work.
        driverBinding = new WasdControllerBinding();
        operatorBinding = new WasdControllerBinding();
        buttonBoardBinding = new WasdControllerBinding();

        driverJoystick = new Joystick(driverPort);
        operatorJoystick = new Joystick(operatorPort);
        buttonBoardJoystick = new Joystick(buttonBoardPort);
    }

    public void init() {
        interface MapButtonFunction<M, B, H, P, R> {
            void apply(M m, B b, H h, P p, R r);
        }

        MapButtonFunction<
                EnumMap<Button, ButtonData>,
                Button,
                Class<? extends ButtonHoldEvent>,
                Class<? extends ButtonPressEvent>,
                Class<? extends ButtonReleaseEvent>
        >
        mapButton = (
                mapping,
                button,
                holdEventClass,
                pressEventClass,
                releaseEventClass
        ) -> {
            ButtonHoldEvent holdEvent = eventAggregator.GetEvent(holdEventClass);
            ButtonPressEvent pressEvent = eventAggregator.GetEvent(pressEventClass);
            ButtonReleaseEvent releaseEvent = eventAggregator.GetEvent(releaseEventClass);

            ButtonData buttonData = new ButtonData();

            buttonData.buttonId = -1;
            buttonData.holdEvent = holdEvent;
            buttonData.pressEvent = pressEvent;
            buttonData.releaseEvent = releaseEvent;

            mapping.put(button, buttonData);
        };

        // Mapping Driver Events
        mapButton.apply(
                driverButtonMappings,
                Button.X,
                DriverXButtonHoldEvent.class,
                DriverXButtonPressEvent.class,
                DriverXButtonReleaseEvent.class
        );

        mapButton.apply(
                driverButtonMappings,
                Button.Y,
                DriverYButtonHoldEvent.class,
                DriverYButtonPressEvent.class,
                DriverYButtonReleaseEvent.class
        );

        for (var button : Button.values()) {
            if (!driverBinding.buttonMap.containsKey(button)) {
                driverButtonMappings.remove(button);
                continue;
            }

            if (!driverButtonMappings.containsKey(button)) {
                continue;
            }

            Integer driverButtonId = driverBinding.buttonMap.get(button);
            ButtonData driverButtonData = driverButtonMappings.get(button);

            driverButtonData.buttonId = driverButtonId;
        }

        // Mapping Operator Events
        mapButton.apply(
                operatorButtonMappings,
                Button.X,
                OperatorXButtonHoldEvent.class,
                OperatorXButtonPressEvent.class,
                OperatorXButtonReleaseEvent.class
        );

        mapButton.apply(
                operatorButtonMappings,
                Button.Y,
                OperatorYButtonHoldEvent.class,
                OperatorYButtonPressEvent.class,
                OperatorYButtonReleaseEvent.class
        );

        // Mapping Button Board Events
        // TODO: Add Button Board Events

    }

    // Driver Events
    public static class DriverXButtonPressEvent extends ButtonPressEvent {}
    public static class DriverXButtonReleaseEvent extends ButtonReleaseEvent {}
    public static class DriverXButtonHoldEvent extends ButtonHoldEvent {}

    public static class DriverYButtonPressEvent extends ButtonPressEvent {}
    public static class DriverYButtonReleaseEvent extends ButtonReleaseEvent {}
    public static class DriverYButtonHoldEvent extends ButtonHoldEvent {}

    // Operator Events
    public static class OperatorXButtonPressEvent extends ButtonPressEvent {}
    public static class OperatorXButtonReleaseEvent extends ButtonReleaseEvent {}
    public static class OperatorXButtonHoldEvent extends ButtonHoldEvent {}

    public static class OperatorYButtonPressEvent extends ButtonPressEvent {}
    public static class OperatorYButtonReleaseEvent extends ButtonReleaseEvent {}
    public static class OperatorYButtonHoldEvent extends ButtonHoldEvent {}

    // Button Board Events


}
