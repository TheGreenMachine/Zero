package com.team1816.lib.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.events.ControllerEvent;
import com.team1816.lib.events.EventAggregator;

import java.util.HashMap;
import java.util.function.Consumer;

/**
 * This class is the main ControlBoard and through a series of hashmaps in ControlBoardBridge associates buttons to actions
 */

@Singleton
public class ControlBoard implements IControlBoard {
    // Note(Michael): I would like to make these protected somehow, but oh well.
    public static class DriverControllerEvent extends ControllerEvent {}
    public static class OperatorControllerEvent extends ControllerEvent {}
    public static class ButtonBoardControllerEvent extends ControllerEvent {}

    private final ControlBoardBridge controlBoardBridge;

    // Control Board
    public static final int kDriveGamepadPort = 0;
    public static final int kOperatorGamepadPort = 1;
    public static final int kButtonBoardPort = 2;

    private EventAggregator eventAggregator;

    private Controller driverController;
    private final ControllerEvent driverControllerEvent;

    private Controller operatorController;
    private final ControllerEvent operatorControllerEvent;
    private final Controller buttonBoardController;
    private final ControllerEvent buttonBoardControllerEvent;

    @Inject
    private ControlBoard(ControlBoardBridge bridge, Controller.Factory controller) {
        eventAggregator = new EventAggregator();

        driverControllerEvent = eventAggregator.GetEvent(DriverControllerEvent.class);
        operatorControllerEvent = eventAggregator.GetEvent(OperatorControllerEvent.class);
        buttonBoardControllerEvent = eventAggregator.GetEvent(ButtonBoardControllerEvent.class);

        driverController = controller.getControllerInstance(kDriveGamepadPort, bridge.getDriverControllerType());
        operatorController = controller.getControllerInstance(kOperatorGamepadPort, bridge.getOperatorControllerType());
        buttonBoardController = new ButtonboardController(kButtonBoardPort);
        controlBoardBridge = bridge;
    }

    public void setDriverControllerType(Controller.Type controllerType) {
        driverController = switch (controllerType) {
            case WASD -> new WasdController(kDriveGamepadPort);
            case XBOX -> new XboxController(kDriveGamepadPort);
            case LOGITECH -> new LogitechController(kDriveGamepadPort);
            case BUTTON_BOARD -> new ButtonboardController(kDriveGamepadPort);
        };
    }

    public void setOperatorControllerType(Controller.Type controllerType) {
        operatorController = switch (controllerType) {
            case WASD -> new WasdController(kOperatorGamepadPort);
            case XBOX -> new XboxController(kOperatorGamepadPort);
            case LOGITECH -> new LogitechController(kOperatorGamepadPort);
            case BUTTON_BOARD -> new ButtonboardController(kOperatorGamepadPort);
        };
    }

    public void addActionToDriver(Consumer<Controller> action) {
        driverControllerEvent.Subscribe(action);
    }

    public void addActionToOperator(Consumer<Controller> action) {
        operatorControllerEvent.Subscribe(action);
    }

    public void addActionToButtonBoard(Consumer<Controller> action) {
        buttonBoardControllerEvent.Subscribe(action);
    }

    public void update() {
        driverControllerEvent.Publish(driverController);
        operatorControllerEvent.Publish(operatorController);
        buttonBoardControllerEvent.Publish(buttonBoardController);
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
