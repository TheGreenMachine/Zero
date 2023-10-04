package com.team1816.lib.input_handler;

import com.google.inject.Inject;
import com.team1816.lib.Injector;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;

public class InputHandlerBridge {
    private InputHandlerConfig config;

    private IControllerBinding driverBinding;
    private IControllerBinding operatorBinding;

    public IControllerBinding getDriverControllerBinding() {
        return driverBinding;
    }

    public IControllerBinding getOperatorControllerBinding() {
        return operatorBinding;
    }

    private IControllerBinding stringToControllerBinding(String nameType) {
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

        if (config != null) {
            if (config.driver != null) {
                if (config.driver.controllerType != null) {
                    driverBinding = stringToControllerBinding(config.driver.controllerType);
                }

                if (config.driver.rumble != null) {
                    // TODO: do something here in the future.
                }
            }

            if (config.operator != null) {
                if (config.operator.controllerType != null) {
                    operatorBinding = stringToControllerBinding(config.operator.controllerType);
                }

                if (config.driver.rumble != null) {
                    // TODO: Do something here in the future.
                }
            }
        }
    }
}
