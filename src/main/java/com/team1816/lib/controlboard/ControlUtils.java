package com.team1816.lib.controlboard;

import com.team1816.lib.util.team254.LatchedBoolean;
import com.team1816.season.Robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * This class is a utilities class for creating controls such as the HoldAction and toggle Action and
 * is responsible for taking lambda expressions as BooleanSuppliers and appending a Runnable action with them
 *
 * @see Robot#robotInit()
 */
public class ControlUtils implements Controller.Factory {

    @Override
    public Controller getControllerInstance(int port) {
        var hid = new Joystick(port);
        var axisCount = hid.getAxisCount();
        if (axisCount <= 3 && RobotBase.isSimulation()) {
            System.out.println("    Using Wasd Controller for port: " + port);
            return new WasdController(port);
        } else if (axisCount == 4) {
            System.out.println("    Using Logitech Controller for port: " + port);
            return new LogitechController(port);
        } else {
            System.out.println("    Using XboxController Controller for port: " + port);
            return new XboxController(port);
        }
    }

    public static PressAction createAction(BooleanSupplier input, Runnable action) {
        return new PressAction(input, action);
    }

    public static HoldAction createHoldAction(
        BooleanSupplier input,
        Consumer<Boolean> action
    ) {
        return new HoldAction(input, action);
    }

    public static ScalarAction createScalar(double input, DoubleConsumer output) {
        return new ScalarAction(input, output);
    }

    public interface ButtonAction {
        void update();
    }

    public static class PressAction implements ButtonAction {

        private final LatchedBoolean pressedState = new LatchedBoolean();
        private final LatchedBoolean releasedState = new LatchedBoolean();
        private final BooleanSupplier input;
        private final Runnable action;

        private PressAction(boolean input, Runnable action) {
            this.input = () -> input;
            this.action = action;
        }

        private PressAction(BooleanSupplier input, Runnable action) {
            this.input = input;
            this.action = action;
        }

        @Override
        public void update() {
            boolean inputPressed = input.getAsBoolean();
            boolean inputJustPressed = pressedState.update(inputPressed);
            boolean inputJustReleased = releasedState.update(!inputPressed);

            if (inputJustPressed) {
                action.run();
            }
            if (inputJustReleased) {
                pressedState.update(false);
            }
        }
    }

    public static class HoldAction implements ButtonAction {

        private BooleanSupplier input;
        private Consumer<Boolean> action;
        private LatchedBoolean pressedState = new LatchedBoolean();
        private LatchedBoolean releasedState = new LatchedBoolean();

        private HoldAction(BooleanSupplier input, Consumer<Boolean> action) {
            this.input = input;
            this.action = action;
        }

        @Override
        public void update() {
            boolean inputPressed = input.getAsBoolean();
            boolean inputJustPressed = pressedState.update(inputPressed);
            boolean inputJustReleased = releasedState.update(!inputPressed);

            if (inputJustPressed) {
                action.accept(true);
            } else if (inputJustReleased) {
                action.accept(false);
            }
        }
    }

    public static class ScalarAction implements ButtonAction {

        private DoubleSupplier input;
        private DoubleConsumer action;

        private double lastValue;

        private ScalarAction(double input, DoubleConsumer action) {
            this.input = () -> input;
            this.action = action;
        }

        @Override
        public void update() {
            double newValue = input.getAsDouble();
            if (newValue != lastValue) {
                action.accept(newValue);
                lastValue = newValue;
            }
        }
    }
}
