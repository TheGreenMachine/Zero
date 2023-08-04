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
        HashMap<String, Controller.Axis>[] axis_maps = new HashMap[]{
                controlBoardBridge.getDriverAxisMap(),
                controlBoardBridge.getOperatorAxisMap()
        };

        HashMap<String, Controller.Button>[] button_maps = new HashMap[]{
                controlBoardBridge.getDriverButtonMap(),
                controlBoardBridge.getOperatorButtonMap(),
                controlBoardBridge.getButtonBoardMap()
        };

        for (var axis_map : axis_maps) {
            if (axis_map.containsKey(name)) {
                return driverController.getJoystick(axis_map.get(name));
            }
        }

        for (var button_map : button_maps) {
            if (button_map.containsKey(name)) {
                return driverController.getButton(button_map.get(name)) ? 1 : 0;
            }
        }

        return defaultVal;
    }

    public boolean getBooleanFromControllerYaml(String name, boolean defaultVal) {
        HashMap<String, Controller.Axis>[] axis_maps = new HashMap[]{
                controlBoardBridge.getDriverAxisMap(),
                controlBoardBridge.getOperatorAxisMap()
        };

        HashMap<String, Controller.Button>[] button_maps = new HashMap[]{
                controlBoardBridge.getDriverButtonMap(),
                controlBoardBridge.getOperatorButtonMap(),
                controlBoardBridge.getButtonBoardMap()
        };

        HashMap<String, Integer>[] dpad_maps = new HashMap[] {
                controlBoardBridge.getDriverDpadMap(),
                controlBoardBridge.getOperatorDpadMap(),
        };

        for (var axis_map : axis_maps) {
            if (axis_map.containsKey(name)) {
                return driverController.getTrigger(axis_map.get(name));
            }
        }

        for (var button_map : button_maps) {
            if (button_map.containsKey(name)) {
                return driverController.getButton(button_map.get(name));
            }
        }

        for (var dpad_map : dpad_maps) {
            if (dpad_map.containsKey(name)) {
                return driverController.getDPad() == dpad_map.get(name);
            }
        }

        return defaultVal;
    }
}
