package com.team1816.lib.input_handler;

import com.team1816.lib.input_handler.bindings.ButtonBoardControllerBinding;
import com.team1816.lib.input_handler.bindings.ControllerBinding;
import com.team1816.lib.input_handler.controlOptions.*;
import com.team1816.lib.input_handler.events.AxisEvent;
import com.team1816.lib.input_handler.events.ButtonEvent;
import com.team1816.lib.input_handler.events.DpadEvent;
import com.team1816.lib.input_handler.events.TriggerEvent;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.EnumMap;
import java.util.function.Consumer;

/**
 * A single object that facilities the assignment of actions to a
 * specific button, axis, and dpad events.
 *
 * @see ControllerBinding
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
        public ControllerBinding binding;
        public ControllerMappingInfo mappingInfo;
        public final EnumMap<Button, ButtonEvent> buttonEventMapping = new EnumMap<>(Button.class);
        public final EnumMap<Axis, AxisEvent> axisEventMapping = new EnumMap<>(Axis.class);
        public final EnumMap<Dpad, DpadEvent> dpadEventMapping = new EnumMap<>(Dpad.class);
        public final EnumMap<Trigger, TriggerEvent> triggerEventMapping = new EnumMap<>(Trigger.class);
    }

    public enum RumbleDirection {
        UNIFORM,
        LEFT,
        RIGHT,
    }

    public enum ControllerRole {
        DRIVER,
        OPERATOR,
        BUTTONBOARD,
    }

    private Controller driver;
    private Controller operator;
    private Controller buttonBoard;

    public static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;
    private static final int BUTTON_BOARD_PORT = 2;

    private Controller[] controllers = new Controller[3];

    private boolean driverRumbleEnabled, operatorRumbleEnabled;

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

        driver.mappingInfo = bridge.getDriverControllerInfo();
        operator.mappingInfo = bridge.getOperatorControllerInfo();
        buttonBoard.mappingInfo = bridge.getButtonBoardControllerInfo();

        driverRumbleEnabled = bridge.isDriverRumbleEnabled();
        operatorRumbleEnabled = bridge.isOperatorRumbleEnabled();

        init();
    }

    /**
     * This procedure listens to any Button, Trigger, or Dpad press.
     *
     * @param mappingName
     * @param state
     * @param action
     */
    public void listenActionButton(String mappingName, ActionState state, Runnable action) {
        for (Controller controller : controllers) {
            Button button = controller.mappingInfo.buttons.get(mappingName);

            if (button != null) {
                listenButton(controller, button, state, action);
            }

            Trigger trigger = controller.mappingInfo.triggers.get(mappingName);

            if (trigger != null) {
                listenTrigger(controller, trigger, state, action);
            }

            Dpad dpad = controller.mappingInfo.dpad.get(mappingName);

            if (dpad != null) {
                listenDpad(controller, dpad, state, action);
            }
        }
    }

    /**
     * This procedure listens to any Axis event
     *
     * @param mappingName
     * @param action
     */
    public void listenActionAxis(String mappingName, Consumer<Double> action) {
        for (Controller controller : controllers) {
           Axis axis = controller.mappingInfo.joysticks.get(mappingName);

           if (axis != null) {
               listenAxis(controller, axis, action);
           }
        }
    }

    public void listenActionPressAndRelease(String mappingName, Consumer<Boolean> action) {
        for (Controller controller : controllers) {
            Button button = controller.mappingInfo.buttons.get(mappingName);

            if (button != null) {
                listenButtonPressAndRelease(controller, button, action);
            }

            Trigger trigger = controller.mappingInfo.triggers.get(mappingName);

            if (trigger != null) {
                listenTriggerPressAndRelease(controller, trigger, action);
            }

            Dpad dpad = controller.mappingInfo.dpad.get(mappingName);

            if (dpad != null) {
                listenDpadPressAndRelease(controller, dpad, action);
            }
        }
    }

    /**
     * Goes through all the possible action names that are bound to
     * a specific button, trigger, axis, or dpad, and returns a double
     * precision floating point value.
     *
     * @param mappingName
     * @return
     */
    public Double getActionAsDouble(String mappingName) {
        for (Controller controller : controllers) {
            Button button = controller.mappingInfo.buttons.get(mappingName);

            if (button != null) {
                return getControllerButtonAsBool(controller, button) ? 1.0 : 0.0;
            }

            Trigger trigger = controller.mappingInfo.triggers.get(mappingName);

            if (trigger != null) {
                return getControllerTriggerAsDouble(controller, trigger);
            }

            Axis axis = controller.mappingInfo.joysticks.get(mappingName);

            if (axis != null) {
                return getControllerAxisAsDouble(controller, axis);
            }

            Dpad dpad = controller.mappingInfo.dpad.get(mappingName);

            if (dpad != null) {
                return getControllerDpadAsBool(controller, dpad) ? 1.0 : 0.0;
            }
        }

        return 0.0;
    }

    /**
     * Goes through all the possible action names that are bound to
     * a specific button, trigger, axis, or dpad, and returns a boolean.
     *
     * @param mappingName
     * @return
     */
    public Boolean getActionAsBool(String mappingName) {
        return getActionAsDouble(mappingName) > Trigger.kAxisThreshold;
    }

    /** Helper function to get a controller button as a boolean */
    private boolean getControllerButtonAsBool(Controller controller, Button button) {
        Integer id = controller.binding.buttonMap.get(button);

        if (id == null) return false;

        return controller.joystick.getRawButton(id);
    }

    /** Helper function to get a controller axis as a double */
    private double getControllerAxisAsDouble(Controller controller, Axis axis) {
        Integer id = controller.binding.axisMap.get(axis);

        if (id == null) return 0.0;

        return controller.joystick.getRawAxis(id);
    }

    /** Helper function to get a controller trigger (axis) as a double */
    private double getControllerTriggerAsDouble(Controller controller, Trigger trigger) {
        Integer id = controller.binding.triggerMap.get(trigger);

        if (id == null) return 0.0;

        return controller.joystick.getRawAxis(id);
    }

    /** Helper function to get a controller trigger (axis) as d boolean */
    private boolean getControllerTriggerAsBool(Controller controller, Trigger trigger) {
        Integer id = controller.binding.triggerMap.get(trigger);

        if (id == null) return false;

        return controller.joystick.getRawAxis(id) > Trigger.kAxisThreshold;
    }

    /** Helper function to get a dpad button as boolean */
    private boolean getControllerDpadAsBool(Controller controller, Dpad dpad) {
        return controller.joystick.getPOV() == dpad.value;
    }

    /** Helper function for listening to a specific button. */
    private void listenButton(
            Controller controller,
            Button button,
            ActionState state,
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
        listenButton(controller, button, ActionState.PRESSED, () -> action.accept(true));
        listenButton(controller, button, ActionState.RELEASED, () -> action.accept(false));
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

    /** Helper function for listening to a specific dpad button */
    private void listenDpad(
            Controller controller,
            Dpad dpad,
            ActionState state,
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
        listenDpad(controller, dpad, ActionState.PRESSED, () -> action.accept(true));
        listenDpad(controller, dpad, ActionState.RELEASED, () -> action.accept(false));
    }

    /** Helper function for listening to a specific trigger */
    private void listenTrigger(
            Controller controller,
            Trigger trigger,
            ActionState state,
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
        listenTrigger(controller, trigger, ActionState.PRESSED, () -> action.accept(true));
        listenTrigger(controller, trigger, ActionState.RELEASED, () -> action.accept(false));
    }

    public void setRumble(ControllerRole role, RumbleDirection direction, double demand) {
        GenericHID.RumbleType rumbleType = switch (direction) {
            case UNIFORM -> GenericHID.RumbleType.kBothRumble;
            case LEFT -> GenericHID.RumbleType.kLeftRumble;
            case RIGHT -> GenericHID.RumbleType.kRightRumble;
        };

        switch (role) {
            case DRIVER:
                driver.joystick.setRumble(rumbleType, demand);
                break;

            case OPERATOR:
                operator.joystick.setRumble(rumbleType, demand);
                break;
        }
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