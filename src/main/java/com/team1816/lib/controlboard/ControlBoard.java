package com.team1816.lib.controlboard;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * This class is the main ControlBoard and through a series of hashmaps in ControlBoardBridge associates buttons to actions
 */
@Singleton
public class ControlBoard implements IControlBoard {

    private final ControlBoardBridge controlBoardBridge;

    // Control Board
    public static final int kDriveGamepadPort = 0;
    public static final int kOperatorGamepadPort = 1;
    public static final int kMrButtonsGamepadPort = 2;

    private final Controller driverController;
    private final Controller operatorController;

    private final Controller mrButtonsController; //Blame Adele She named the operator Button Board

    @Inject
    private ControlBoard(ControlBoardBridge bridge, Controller.Factory controller) {
        driverController = controller.getControllerInstance(kDriveGamepadPort);
        operatorController = controller.getControllerInstance(kOperatorGamepadPort);
        mrButtonsController = controller.getControllerInstance(kMrButtonsGamepadPort);
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
        if (controlBoardBridge.driverMapContainsKey(name)) {
            if (controlBoardBridge.getDriverAxisMap().containsKey(name)) {
                return driverController.getJoystick(
                    controlBoardBridge.getDriverAxisMap().get(name)
                );
            }
            if (controlBoardBridge.getDriverButtonMap().containsKey(name)) {
                return driverController.getButton(
                        controlBoardBridge.getDriverButtonMap().get(name)
                    )
                    ? 1
                    : 0;
            }
        }

        else if (controlBoardBridge.operatorMapContainsKey(name)) {
            if (controlBoardBridge.getOperatorAxisMap().containsKey(name)) {
                return operatorController.getJoystick(
                    controlBoardBridge.getOperatorAxisMap().get(name)
                );
            } else if (controlBoardBridge.getOperatorButtonMap().containsKey(name)) {
                return operatorController.getButton(
                        controlBoardBridge.getOperatorButtonMap().get(name)
                    )
                    ? 1
                    : 0;
            }
        }

        else if (controlBoardBridge.mrButtonsMapContrainsKey(name)) {
            if (controlBoardBridge.getMrButtonAxisMap().containsKey(name)) {
                return operatorController.getJoystick(
                    controlBoardBridge.getMrButtonAxisMap().get(name)
                );
            } else if (controlBoardBridge.getMrButtons_ButtonMap().containsKey(name)) {
                return operatorController.getButton(
                    controlBoardBridge.getMrButtons_ButtonMap().get(name)
                )
                    ? 1
                    : 0;
            }
        }





        return defaultVal;
    }

    public boolean getBooleanFromControllerYaml(String name, boolean defaultVal) {
        if (controlBoardBridge.driverMapContainsKey(name)) {
            if (controlBoardBridge.getDriverAxisMap().containsKey(name)) {
                return driverController.getTrigger(
                    controlBoardBridge.getDriverAxisMap().get(name)
                );
            } else if (controlBoardBridge.getDriverButtonMap().containsKey(name)) {
                return driverController.getButton(
                    controlBoardBridge.getDriverButtonMap().get(name)
                );
            } else if (controlBoardBridge.getDriverDpadMap().containsKey(name)) {
                return (
                    driverController.getDPad() ==
                    controlBoardBridge.getDriverDpadMap().get(name)
                );
            }
        }
        else if (controlBoardBridge.operatorMapContainsKey(name)) {
            if (controlBoardBridge.getOperatorAxisMap().containsKey(name)) {
                return operatorController.getTrigger(
                    controlBoardBridge.getOperatorAxisMap().get(name)
                );
            } else if (controlBoardBridge.getOperatorButtonMap().containsKey(name)) {
                return operatorController.getButton(
                    controlBoardBridge.getOperatorButtonMap().get(name)
                );
            } else if (controlBoardBridge.getOperatorDpadMap().containsKey(name)) {
                return (
                    operatorController.getDPad() ==
                    controlBoardBridge.getOperatorDpadMap().get(name)
                );
            }
        }
        else if (controlBoardBridge.mrButtonsMapContrainsKey(name)) {
            if (controlBoardBridge.getMrButtonAxisMap().containsKey(name)) {
                return mrButtonsController.getTrigger(
                    controlBoardBridge.getMrButtonAxisMap().get(name)
                );
            } else if (controlBoardBridge.getMrButtons_ButtonMap().containsKey(name)) {
                return operatorController.getButton(
                    controlBoardBridge.getMrButtons_ButtonMap().get(name)
                );
            } else if (controlBoardBridge.getMrButtonsDpadMap().containsKey(name)) {
                return (
                    mrButtonsController.getDPad() ==
                        controlBoardBridge.getMrButtonsDpadMap().get(name)
                );
            }
        }

        return defaultVal;
    }
}
