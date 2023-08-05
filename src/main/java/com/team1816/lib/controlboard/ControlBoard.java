package com.team1816.lib.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import java.util.HashMap;

/**
 * This class is the main ControlBoard and through a series of hashmaps in ControlBoardBridge associates buttons to actions
 */

@Singleton
public class ControlBoard implements IControlBoard {

    private final ControlBoardBridge controlBoardBridge;

    // Control Board
    public static final int kDriveGamepadPort = 0;
    public static final int kOperatorGamepadPort = 1;
    public static final int kButtonBoardPort = 2;

    public final Controller driverController;
    public final Controller operatorController;

    public final Controller buttonBoardController;

    @Inject
    private ControlBoard(ControlBoardBridge bridge, Controller.Factory controller) {
        driverController = controller.getControllerInstance(kDriveGamepadPort, bridge.getDriverControllerType());
        operatorController = controller.getControllerInstance(kOperatorGamepadPort, bridge.getOperatorControllerType());
        buttonBoardController = controller.getControllerInstance(kButtonBoardPort, "ButtonBoard");
        controlBoardBridge = bridge;
    }

    @Override
    public boolean getAsBool(String getName) {
        return getBooleanFromControllerYaml(getName);
    }

    @Override
    public double getAsDouble(String getName) {
        return getDoubleFromControllerYaml(getName);
    }

    public double getDoubleFromControllerYaml(String name) {
        return getDoubleFromControllerYaml(name, 0);
    }

    public boolean getBooleanFromControllerYaml(String name) {
        return getBooleanFromControllerYaml(name, false);
    }

    public double getDoubleFromControllerYaml(String name, double defaultVal) {
        HashMap<String, Controller.Axis>[] axisMaps = new HashMap[]{
                controlBoardBridge.getDriverAxisMap(),
                controlBoardBridge.getOperatorAxisMap()
        };

        HashMap<String, Controller.Button>[] buttonMaps = new HashMap[]{
                controlBoardBridge.getDriverButtonMap(),
                controlBoardBridge.getOperatorButtonMap(),
                controlBoardBridge.getButtonBoardMap()
        };

        Controller[] controllers = new Controller[] {
                driverController,
                operatorController,
                buttonBoardController
        };

        for (var i = 0; i < axisMaps.length; i++) {
            var axisMap = axisMaps[i];
            var controller = controllers[i];

            if (axisMap.containsKey(name)) {
                return controller.getJoystick(axisMap.get(name));
            }
        }

        for (var i = 0; i < buttonMaps.length; i++) {
            var buttonMap = buttonMaps[i];
            var controller = controllers[i];

            if (buttonMap.containsKey(name)) {
                return controller.getButton(buttonMap.get(name)) ? 1 : 0;
            }
        }

        return defaultVal;
    }

    public boolean getBooleanFromControllerYaml(String name, boolean defaultVal) {
        HashMap<String, Controller.Axis>[] axisMaps = new HashMap[]{
                controlBoardBridge.getDriverAxisMap(),
                controlBoardBridge.getOperatorAxisMap()
        };

        HashMap<String, Controller.Button>[] buttonMaps = new HashMap[]{
                controlBoardBridge.getDriverButtonMap(),
                controlBoardBridge.getOperatorButtonMap(),
                controlBoardBridge.getButtonBoardMap()
        };

        HashMap<String, Integer>[] dpadMaps = new HashMap[] {
                controlBoardBridge.getDriverDpadMap(),
                controlBoardBridge.getOperatorDpadMap(),
        };

        Controller[] controllers = new Controller[] {
                driverController,
                operatorController,
                buttonBoardController
        };

        for (var i = 0; i < axisMaps.length; i++) {
            var axisMap = axisMaps[i];
            var controller = controllers[i];

            if (axisMap.containsKey(name)) {
                return controller.getTrigger(axisMap.get(name));
            }
        }

        for (var i = 0; i < buttonMaps.length; i++) {
            var buttonMap = buttonMaps[i];
            var controller = controllers[i];

            if (buttonMap.containsKey(name)) {
                return controller.getButton(buttonMap.get(name));
            }
        }

        for (var i = 0; i < dpadMaps.length; i++) {
            var dpadMap = dpadMaps[i];
            var controller = controllers[i];

            if (dpadMap.containsKey(name)) {
                return controller.getDPad() == dpadMap.get(name);
            }
        }

        return defaultVal;
    }
}
