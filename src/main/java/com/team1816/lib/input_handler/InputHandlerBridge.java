package com.team1816.lib.input_handler;

import com.google.inject.Inject;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;

public class InputHandlerBridge {
    private InputHandlerConfig config;

    private ControllerBinding driverBinding;
    private ControllerBinding operatorBinding;

    public ControllerBinding getDriverControllerBinding() {
        return driverBinding;
    }

    public ControllerBinding getOperatorControllerBinding() {
        return operatorBinding;
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
            config = InputHandlerConfigYaml.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(
                                "yaml/input_handler.config.yml"
                        )
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
