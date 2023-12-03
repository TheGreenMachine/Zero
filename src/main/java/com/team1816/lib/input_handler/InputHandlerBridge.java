package com.team1816.lib.input_handler;

import com.google.inject.Inject;
import com.team1816.lib.Injector;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;

public class InputHandlerBridge {
    private InputHandlerConfig config;

    private ControllerBinding driverBinding;
    private ControllerBinding operatorBinding;

    private ControllerMappingInfo driverInfo;
    private ControllerMappingInfo operatorInfo;
    private ControllerMappingInfo buttonBoardInfo;

    private boolean driverRumble = false;
    private boolean operatorRumble = false;

    public ControllerBinding getDriverControllerBinding() {
        return driverBinding;
    }

    public ControllerBinding getOperatorControllerBinding() {
        return operatorBinding;
    }

    public ControllerMappingInfo getDriverControllerInfo() {
        return driverInfo;
    }

    public ControllerMappingInfo getOperatorControllerInfo() {
        return operatorInfo;
    }

    public ControllerMappingInfo getButtonBoardControllerInfo() {
        return buttonBoardInfo;
    }

    public boolean isDriverRumbleEnabled() {
        return driverRumble;
    }

    public boolean isOperatorRumbleEnabled() {
        return operatorRumble;
    }

    private ControllerBinding stringToControllerBinding(String nameType) {
        switch (nameType) {
            case "Wasd": return new WasdControllerBinding();
            case "Xbox": return new XboxControllerBinding();
            case "ButtonBoard": return new ButtonBoardControllerBinding();
        }

        return new WasdControllerBinding();
    }

    @Inject
    public InputHandlerBridge() {
        try {
            RobotFactory factory = Injector.get(RobotFactory.class);
            String inputHandlerConfigFileName = factory.getInputHandlerName();

            String location =
                    "yaml/input_handler/" +
                            inputHandlerConfigFileName +
                            ".input_handler.config.yml";

            GreenLogger.log("Attempting to load input handler yaml at: " + location);
            config = InputHandlerConfigYaml.loadFrom(
                    this.getClass()
                            .getClassLoader()
                            .getResourceAsStream(location)
            );
        } catch (Exception e) {
            GreenLogger.log(e);
            DriverStation.reportError(
                    "Input Handler Yaml Config error!",
                    e.getStackTrace()
            );
        }

        driverInfo = new ControllerMappingInfo();
        operatorInfo = new ControllerMappingInfo();
        buttonBoardInfo = new ControllerMappingInfo();

        if (config != null) {
            if (config.driver != null) {
                if (config.driver.controllerType != null) {
                    driverBinding = stringToControllerBinding(config.driver.controllerType);
                }

                if (config.driver.axes != null) {
                    config.driver.axes.forEach((key, value) -> {
                        if (key != null) {
                            switch (key) {
                                case "leftTrigger" -> driverInfo.triggers.put(value, Trigger.LEFT);
                                case "rightTrigger" -> driverInfo.triggers.put(value, Trigger.RIGHT);
                            }
                        }
                    });
                }

                if (config.driver.buttons != null) {
                    config.driver.buttons.forEach((key, value) -> {
                        if (key != null) {
                            switch (key) {
                                case "leftBumper" -> driverInfo.buttons.put(value, Button.LEFT_BUMPER);
                                case "rightBumper" -> driverInfo.buttons.put(value, Button.RIGHT_BUMPER);
                                case "start" -> driverInfo.buttons.put(value, Button.START);
                                case "back" -> driverInfo.buttons.put(value, Button.BACK);
                            }
                        }
                    });
                }

                if (config.driver.joysticks != null) {
                    config.driver.joysticks.forEach((key, value) -> {
                        if (key != null) {
                            if (value.horizontal != null) {
                                switch (key) {
                                    case "right" -> driverInfo.joysticks.put(value.horizontal, Axis.RIGHT_HORIZONTAL);
                                    case "left" -> driverInfo.joysticks.put(value.horizontal, Axis.LEFT_HORIZONTAL);
                                }
                            }

                            if (value.vertical != null) {
                                switch (key) {
                                    case "right" -> driverInfo.joysticks.put(value.vertical, Axis.RIGHT_VERTICAL);
                                    case "left" -> driverInfo.joysticks.put(value.vertical, Axis.LEFT_VERTICAL);
                                }
                            }
                        }
                    });
                }

                if (config.driver.buttonpad != null) {
                    ButtonpadConfig buttonpad = config.driver.buttonpad;
                    if (buttonpad.a != null) driverInfo.buttons.put(buttonpad.a, Button.A);
                    if (buttonpad.b != null) driverInfo.buttons.put(buttonpad.b, Button.B);
                    if (buttonpad.x != null) driverInfo.buttons.put(buttonpad.x, Button.X);
                    if (buttonpad.y != null) driverInfo.buttons.put(buttonpad.y, Button.Y);
                }

                if (config.driver.dpad != null) {
                    DpadConfig dpad = config.driver.dpad;
                    if (dpad.up != null) driverInfo.dpad.put(dpad.up, Dpad.UP);
                    if (dpad.down != null) driverInfo.dpad.put(dpad.down, Dpad.DOWN);
                    if (dpad.left != null) driverInfo.dpad.put(dpad.left, Dpad.LEFT);
                    if (dpad.right != null) driverInfo.dpad.put(dpad.right, Dpad.RIGHT);
                }

                if (config.driver.rumble != null) {
                    driverRumble = config.driver.rumble;
                }
            }

            if (config.operator != null) {
                if (config.operator.controllerType != null) {
                    operatorBinding = stringToControllerBinding(config.operator.controllerType);
                }

                if (config.operator.axes != null) {
                    config.operator.axes.forEach((key, value) -> {
                        if (key != null) {
                            switch (key) {
                                case "leftTrigger" -> operatorInfo.triggers.put(value, Trigger.LEFT);
                                case "rightTrigger" -> operatorInfo.triggers.put(value, Trigger.RIGHT);
                            }
                        }
                    });
                }

                if (config.operator.buttons != null) {
                    config.operator.buttons.forEach((key, value) -> {
                        if (key != null) {
                            switch (key) {
                                case "leftBumper" -> operatorInfo.buttons.put(value, Button.LEFT_BUMPER);
                                case "rightBumper" -> operatorInfo.buttons.put(value, Button.RIGHT_BUMPER);
                                case "start" -> operatorInfo.buttons.put(value, Button.START);
                                case "back" -> operatorInfo.buttons.put(value, Button.BACK);
                            }
                        }
                    });
                }

                if (config.operator.joysticks != null) {
                    config.operator.joysticks.forEach((key, value) -> {
                        if (key != null) {
                            if (value.horizontal != null) {
                                switch (key) {
                                    case "right" -> operatorInfo.joysticks.put(value.horizontal, Axis.RIGHT_HORIZONTAL);
                                    case "left" -> operatorInfo.joysticks.put(value.horizontal, Axis.LEFT_HORIZONTAL);
                                }
                            }

                            if (value.vertical != null) {
                                switch (key) {
                                    case "right" -> operatorInfo.joysticks.put(value.vertical, Axis.RIGHT_VERTICAL);
                                    case "left" -> operatorInfo.joysticks.put(value.vertical, Axis.LEFT_VERTICAL);
                                }
                            }
                        }
                    });
                }

                if (config.operator.buttonpad != null) {
                    ButtonpadConfig buttonpad = config.operator.buttonpad;
                    if (buttonpad.a != null) operatorInfo.buttons.put(buttonpad.a, Button.A);
                    if (buttonpad.b != null) operatorInfo.buttons.put(buttonpad.b, Button.B);
                    if (buttonpad.x != null) operatorInfo.buttons.put(buttonpad.x, Button.X);
                    if (buttonpad.y != null) operatorInfo.buttons.put(buttonpad.y, Button.Y);
                }

                if (config.operator.dpad != null) {
                    DpadConfig dpad = config.operator.dpad;
                    if (dpad.up != null) operatorInfo.dpad.put(dpad.up, Dpad.UP);
                    if (dpad.down != null) operatorInfo.dpad.put(dpad.down, Dpad.DOWN);
                    if (dpad.left != null) operatorInfo.dpad.put(dpad.left, Dpad.LEFT);
                    if (dpad.right != null) operatorInfo.dpad.put(dpad.right, Dpad.RIGHT);
                }

                if (config.operator.rumble != null) {
                    operatorRumble = config.operator.rumble;
                }
            }

            if (config.buttonboard != null) {
                config.buttonboard.forEach((key, value) -> {
                    if (key != null) {
                        switch (key) {
                            case "upLeft" -> buttonBoardInfo.buttons.put(value, Button.UP_LEFT);
                            case "up" -> buttonBoardInfo.buttons.put(value, Button.UP);
                            case "upRight" -> buttonBoardInfo.buttons.put(value, Button.UP_RIGHT);
                            case "left" -> buttonBoardInfo.buttons.put(value, Button.LEFT);
                            case "right" -> buttonBoardInfo.buttons.put(value, Button.RIGHT);
                            case "downLeft" -> buttonBoardInfo.buttons.put(value, Button.DOWN_LEFT);
                            case "down" -> buttonBoardInfo.buttons.put(value, Button.DOWN);
                            case "downRight" -> buttonBoardInfo.buttons.put(value, Button.DOWN_RIGHT);
                        }
                    }
                });
            }
        }
    }
}
