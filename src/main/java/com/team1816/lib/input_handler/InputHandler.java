package com.team1816.lib.input_handler;

import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.Joystick;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.EnumMap;
import java.util.function.Consumer;

/**
 * A single object that facilities the assignment of actions to
 * specific button, axis, and dpad events.
 *
 * @see IControllerBinding
 * @see Button
 * @see ButtonEvent
 * @see Axis
 * @see AxisEvent
 * @see Dpad
 * @see DpadEvent
 */
@Singleton
public class InputHandler {
    protected class Controller {
        public Joystick joystick;
        public IControllerBinding binding;
        public final EnumMap<Button, ButtonEvent> buttonEventMapping = new EnumMap<>(Button.class);
        public final EnumMap<Axis, AxisEvent> axisEventMapping = new EnumMap<>(Axis.class);
        public final EnumMap<Dpad, DpadEvent> dpadEventMapping = new EnumMap<>(Dpad.class);
        public final EnumMap<Trigger, TriggerEvent> triggerEventMapping = new EnumMap<>(Trigger.class);
    }

    private Controller driver;
    private Controller operator;
    private Controller buttonBoard;

    public static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;
    private static final int BUTTON_BOARD_PORT = 2;

    private Controller[] controllers = new Controller[3];

    @Inject
    public InputHandler(InputHandlerBridge bridge) {
        driver = new Controller();
        operator = new Controller();
        buttonBoard = new Controller();

        controllers[0] = driver;
        controllers[1] = operator;
        controllers[2] = buttonBoard;

        driver.binding = bridge.getDriverControllerBinding();
        operator.binding = bridge.getOperatorControllerBinding();

        // Permanent solution
        buttonBoard.binding = new ButtonBoardControllerBinding();

        driver.joystick = new Joystick(DRIVER_PORT);
        operator.joystick = new Joystick(OPERATOR_PORT);
        buttonBoard.joystick = new Joystick(BUTTON_BOARD_PORT);

        init();
    }

    /** Helper function to get a controller button as a boolean */
    private boolean getControllerButtonAsBool(Controller controller, Button button) {
        Integer id = controller.binding.buttonMap.get(button);

        if (id == null) return false;

        return controller.joystick.getRawButton(id);
    }

    public boolean getDriverButtonAsBool(Button button) {
        return getControllerButtonAsBool(driver, button);
    }

    public boolean getOperatorButtonAsBool(Button button) {
        return getControllerButtonAsBool(operator, button);
    }

    public boolean getButtonBoardAsBool(Button button) {
        return getControllerButtonAsBool(buttonBoard, button);
    }

    /** Helper function to get a controller axis as a double */
    private double getControllerAxisAsDouble(Controller controller, Axis axis) {
        Integer id = controller.binding.axisMap.get(axis);

        if (axis == Axis.RIGHT_HORIZONTAL) {
            GreenLogger.log("Is the controller an instance of WASD? " + (boolean)(controller.binding instanceof WasdControllerBinding));
            GreenLogger.log("Is there no mapping of RIGHT HORIZONTAL on the controller? " + id);
        }

        if (id == null) return 0.0;

        return controller.joystick.getRawAxis(id);
    }

    public double getDriverAxisAsDouble(Axis axis) {
        return getControllerAxisAsDouble(driver, axis);
    }

    public double getOperatorAxisAsDouble(Axis axis) {
        return getControllerAxisAsDouble(operator, axis);
    }

    /** Helper function to get a controller trigger (axis) as a double */
    private double getControllerTriggerAsDouble(Controller controller, Trigger trigger) {
        Integer id = controller.binding.triggerMap.get(trigger);

        if (id == null) return 0.0;

        return controller.joystick.getRawAxis(id);
    }

    public double getDriverTriggerAsDouble(Trigger trigger) {
        return getControllerTriggerAsDouble(driver, trigger);
    }

    public double getOperatorTriggerAsDouble(Trigger trigger) {
        return getControllerTriggerAsDouble(operator, trigger);
    }

    /** Helper function to get a controller trigger (axis) as d boolean */
    private boolean getControllerTriggerAsBool(Controller controller, Trigger trigger) {
        Integer id = controller.binding.triggerMap.get(trigger);

        if (id == null) return false;

        return controller.joystick.getRawAxis(id) > Trigger.kAxisThreshold;
    }

    public boolean getDriverTriggerAsBool(Trigger trigger) {
        return getControllerTriggerAsBool(driver, trigger);
    }

    public boolean getOperatorTriggerAsBool(Trigger trigger) {
        return getControllerTriggerAsBool(operator, trigger);
    }

    /** Helper function for listening to a specific button. */
    private void listenButton(
            Controller controller,
            Button button,
            Button.State state,
            Runnable action
    ) {
       ButtonEvent event = controller.buttonEventMapping.get(button);

       if (event == null) {
           GreenLogger.log("INPUT HANDLER: Button `" + button.toString() + "` is not mapped!");
           return;
       }

       switch (state) {
           case HELD -> event.addHoldAction(action);
           case PRESSED -> event.addPressAction(action);
           case RELEASED -> event.addReleaseAction(action);
       }
    }

    public void listenDriverButton(Button button, Button.State state, Runnable action) {
        listenButton(driver, button, state, action);
    }

    public void listenOperatorButton(Button button, Button.State state, Runnable action) {
        listenButton(operator, button, state, action);
    }

    public void listenButtonBoardButton(Button button, Button.State state, Runnable action) {
        listenButton(buttonBoard, button, state, action);
    }

    /**
     * A helper function to bind a button to a press and release action.
     *
     * This function will pass in 'true' to the action when the button is
     * initially pressed and 'false' when the button is released.
     */
    private void listenButtonPressAndRelease(
            Controller controller,
            Button button,
            Consumer<Boolean> action
    ) {
        listenButton(controller, button, Button.State.PRESSED, () -> action.accept(true));
        listenButton(controller, button, Button.State.RELEASED, () -> action.accept(false));
    }

    public void listenDriverButtonPressAndRelease(Button button, Consumer<Boolean> action) {
        listenButtonPressAndRelease(driver, button, action);
    }

    public void listenOperatorButtonPressAndRelease(Button button, Consumer<Boolean> action) {
        listenButtonPressAndRelease(operator, button, action);
    }

    public void listenButtonBoardButtonPressAndRelease(Button button, Consumer<Boolean> action) {
        listenButtonPressAndRelease(buttonBoard, button, action);
    }

    /** Helper function for listening to a specific axis. */
    private void listenAxis(
            Controller controller,
            Axis axis,
            Consumer<Double> action
    ) {
        AxisEvent event = controller.axisEventMapping.get(axis);

        if (event == null) {
            GreenLogger.log("INPUT HANDLER: Axis `" + axis.toString() + "` is not mapped!");
            return;
        }

        event.addAction(action);
    }

    public void listenDriverAxis(Axis axis, Consumer<Double> action) {
        listenAxis(driver, axis, action);
    }

    public void listenOperatorAxis(Axis axis, Consumer<Double> action) {
        listenAxis(operator, axis, action);
    }

    /** Helper function for listening to a specific dpad button */
    private void listenDpad(
            Controller controller,
            Dpad dpad,
            Dpad.State state,
            Runnable action
    ) {
        DpadEvent event = controller.dpadEventMapping.get(dpad);

        if (event == null) {
            GreenLogger.log("INPUT HANDLER: Dpad Button `" + dpad.toString() + "` is not mapped!");
            return;
        }

        switch (state) {
            case PRESSED -> event.addPressAction(action);
            case HELD -> event.addHoldAction(action);
            case RELEASED -> event.addReleaseAction(action);
        }
    }

    public void listenDriverDpad(Dpad dpad, Dpad.State state, Runnable action) {
        listenDpad(driver, dpad, state, action);
    }

    public void listenOperatorDpad(Dpad dpad, Dpad.State state, Runnable action) {
        listenDpad(operator, dpad, state, action);
    }

    /**
     * A helper function to bind a direction on the dpad to a press and release action.
     *
     * @see #listenButtonPressAndRelease
     */
    private void listenDpadPressAndRelease(
            Controller controller,
            Dpad dpad,
            Consumer<Boolean> action
    ) {
        listenDpad(controller, dpad, Dpad.State.PRESSED, () -> action.accept(true));
        listenDpad(controller, dpad, Dpad.State.RELEASED, () -> action.accept(false));
    }

    public void listenDriverDpadPressAndRelease(Dpad dpad, Consumer<Boolean> action) {
        listenDpadPressAndRelease(driver, dpad, action);
    }

    public void listenOperatorDpadPressAndRelease(Dpad dpad, Consumer<Boolean> action) {
        listenDpadPressAndRelease(operator, dpad, action);
    }

    /** Helper function for listening to a specific trigger */
    private void listenTrigger(
            Controller controller,
            Trigger trigger,
            Trigger.State state,
            Runnable action
    ) {
        TriggerEvent event = controller.triggerEventMapping.get(trigger);

        if (event == null) {
            GreenLogger.log("INPUT HANDLER: Trigger Button `" + trigger.toString() + "` is not mapped!");
            return;
        }

        switch (state) {
            case PRESSED -> event.addPressAction(action);
            case HELD -> event.addHoldAction(action);
            case RELEASED -> event.addReleaseAction(action);
        }
    }

    public void listenDriverTrigger(Trigger trigger, Trigger.State state, Runnable action) {
        listenTrigger(driver, trigger, state, action);
    }

    public void listenOperatorTrigger(Trigger trigger, Trigger.State state, Runnable action) {
        listenTrigger(operator, trigger, state, action);
    }

    /**
     * A helper function to bind a trigger to a press and release action.
     *
     * @see #listenButtonPressAndRelease
     */
    private void listenTriggerPressAndRelease(
            Controller controller,
            Trigger trigger,
            Consumer<Boolean> action
    ) {
        listenTrigger(controller, trigger, Trigger.State.PRESSED, () -> action.accept(true));
        listenTrigger(controller, trigger, Trigger.State.RELEASED, () -> action.accept(false));
    }

    public void listenDriverTriggerPressAndRelease(Trigger trigger, Consumer<Boolean> action) {
        listenTriggerPressAndRelease(driver, trigger, action);
    }

    public void listenOperatorTriggerPressAndRelease(Trigger trigger, Consumer<Boolean> action) {
        listenTriggerPressAndRelease(operator, trigger, action);
    }

    public void init() {
        for (Controller controller : controllers) {
            // Mapping a specific button to an event via an id from the binding.
            controller.binding.buttonMap.forEach((button, id) -> {
                ButtonEvent event = new ButtonEvent(id);

                controller.buttonEventMapping.put(button, event);
            });

            // Mapping a specific axis to an event via an id from the binding.
            controller.binding.axisMap.forEach((axis, id) -> {
               AxisEvent event = new AxisEvent(id);

               controller.axisEventMapping.put(axis, event);
            });

            // Mapping a specific trigger to an event via an id from the binding.
            controller.binding.triggerMap.forEach((trigger, id) -> {
                TriggerEvent event = new TriggerEvent(id);

                controller.triggerEventMapping.put(trigger, event);
            });

            for (Dpad dpad : Dpad.values()) {
                controller.dpadEventMapping.put(dpad, new DpadEvent());
            }
        }
    }

    public void update() {
        for (Controller controller : controllers) {
            controller.buttonEventMapping.forEach((button, event) -> {
                boolean held = controller.joystick.getRawButton(event.getId());

                event.publish(held);
            });

            controller.axisEventMapping.forEach((axis, event) -> {
                double value = controller.joystick.getRawAxis(event.getId());

                event.publish(value);
            });

            controller.triggerEventMapping.forEach((trigger, event) -> {
                double value = controller.joystick.getRawAxis(event.getId());

                event.publish(value > Trigger.kAxisThreshold);
            });

            int dpadValue = controller.joystick.getPOV();

            controller.dpadEventMapping.forEach((dpad, event) -> {
                event.publish(dpad.value == dpadValue);
            });
        }
    }
}