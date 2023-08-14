package com.team1816.lib.input_handler;

import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.Joystick;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.EnumMap;
import java.util.function.Consumer;

@Singleton
public class InputHandler {
    protected class Controller {
        public Joystick joystick;
        public ControllerBinding binding;
        public final EnumMap<Button, ButtonEvent> buttonEventMapping = new EnumMap<>(Button.class);
        public final EnumMap<Axis, AxisEvent> axisEventMapping = new EnumMap<>(Axis.class);
        public final EnumMap<Dpad, DpadEvent> dpadEventMapping = new EnumMap<>(Dpad.class);
    }

    // Note(Michael): Potentially rename `driver` to `main`?
    private Controller driver;
    private Controller operator;
    private Controller buttonBoard;

    public static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;
    private static final int BUTTON_BOARD_PORT = 2;

    private Controller[] controllers = new Controller[3];

    @Inject
    public InputHandler() {
        driver = new Controller();
        operator = new Controller();
        buttonBoard = new Controller();

        // Note(Michael): I could make the controllers be an enum map to hold
        // which controller the thing belongs to.
        //
        // For Example:
        // DriverType.DRIVER
        // DriverType.OPERATOR
        // DriverType.BUTTON_BOARD
        controllers[0] = driver;
        controllers[1] = operator;
        controllers[2] = buttonBoard;

        // Temp solution
        driver.binding = new WasdControllerBinding();
        operator.binding = new WasdControllerBinding();

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

        if (id == null) return 0.0;

        return controller.joystick.getRawAxis(id);
    }

    public double getDriverAxisAsDouble(Axis axis) {
        return getControllerAxisAsDouble(driver, axis);
    }

    public double getOperatorAxisAsDouble(Axis axis) {
        return getControllerAxisAsDouble(operator, axis);
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
           GreenLogger.log("    INPUT HANDLER: Button `" + button.toString() + "` is not mapped!");
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

    /** Helper function for listening to a specific axis. */
    private void listenAxis(
            Controller controller,
            Axis axis,
            Consumer<Double> action
    ) {
        AxisEvent event = controller.axisEventMapping.get(axis);

        if (event == null) {
            GreenLogger.log("   INPUT HANDLER: Axis `" + axis.toString() + "` is not mapped!");
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
            GreenLogger.log("   INPUT HANDLER: Dpad Button `" + dpad.toString() + "` is not mapped!");
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

    public void init() {
        for (var controller : controllers) {
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

            for (var dpad : Dpad.values()) {
                controller.dpadEventMapping.put(dpad, new DpadEvent());
            }
        }
    }

    public void update() {
        for (var controller : controllers) {
            controller.buttonEventMapping.forEach((button, event) -> {
                boolean held = controller.joystick.getRawButton(event.getId());

                event.publish(held);
            });

            controller.axisEventMapping.forEach((axis, event) -> {
                double value = controller.joystick.getRawAxis(event.getId());

                event.publish(value);
            });

            int dpadValue = controller.joystick.getPOV();

            controller.dpadEventMapping.forEach((dpad, event) -> {
                event.publish(dpad.value == dpadValue);
            });
        }
    }

}
