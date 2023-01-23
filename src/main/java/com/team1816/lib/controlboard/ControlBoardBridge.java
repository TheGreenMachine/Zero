package com.team1816.lib.controlboard;

import com.google.inject.Inject;
import com.team1816.lib.Injector;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.season.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;

/**
 * A finite bridging class that allows for yaml based control mappings and distribution in a driver-operator system.
 * @see Robot#robotInit()
 * @see ControlBoard
 */
public class ControlBoardBridge {

    /** Configuration */
    public static RobotFactory factory;
    private ControlBoardConfig config;

    /** State */
    private HashMap<String, Controller.Button> driverButtonMap = new HashMap<>();
    private HashMap<String, Controller.Axis> driverAxisMap = new HashMap<>();
    private HashMap<String, Integer> driverDpadMap = new HashMap<>();
    private boolean driverRumble = false;

    private HashMap<String, Controller.Button> operatorButtonMap = new HashMap<>();
    private HashMap<String, Controller.Axis> operatorAxisMap = new HashMap<>();
    private HashMap<String, Integer> operatorDpadMap = new HashMap<>();
    private boolean operatorRumble = false;

    private HashMap<String, Controller.Button> mrButtons_ButtonMap = new HashMap<>();
    private HashMap<String, Controller.Axis> mrButtonAxisMap = new HashMap<>();
    private HashMap<String, Integer> mrButtonsDpadMap = new HashMap<>();

    private boolean mrButtonsRumble = false;

    /**
     * Instantiates the ControlBoardBridge which maps controllable axes and buttons
     */
    @Inject
    public ControlBoardBridge() {
        factory = Injector.get(RobotFactory.class);
        var controlBoardConfigName = factory.getControlBoard();
        if (controlBoardConfigName.equals("empty")) {
            DriverStation.reportError(
                "Control Board is not defined in Robot Config yaml",
                new NullPointerException().getStackTrace()
            );
            controlBoardConfigName = "example";
        }
        try {
            config =
                ControlBoardYamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream(
                            "yaml/controlboard/" +
                            controlBoardConfigName +
                            ".controlboard.config.yml"
                        )
                );
            System.out.println(
                "Loading \"" + controlBoardConfigName + "\" control board config"
            );
        } catch (Exception e) {
            System.out.println(e);
            DriverStation.reportError(
                "Control Board Yaml Config error!",
                e.getStackTrace()
            );
        }

        if (config != null) {
            if (config.driver != null) {
                if (config.driver.rumble != null) {
                    driverRumble = config.driver.rumble;
                }
                if (config.driver.joysticks != null) {
                    if (config.driver.joysticks.get("left") != null) {
                        if (config.driver.joysticks.get("left").horizontal != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("left").horizontal,
                                Controller.Axis.LEFT_X
                            );
                        }
                        if (config.driver.joysticks.get("left").vertical != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("left").vertical,
                                Controller.Axis.LEFT_Y
                            );
                        }
                    }
                    if (config.driver.joysticks.get("right") != null) {
                        if (config.driver.joysticks.get("right").horizontal != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("right").horizontal,
                                Controller.Axis.RIGHT_X
                            );
                        }
                        if (config.driver.joysticks.get("right").vertical != null) {
                            driverAxisMap.put(
                                config.driver.joysticks.get("right").vertical,
                                Controller.Axis.RIGHT_Y
                            );
                        }
                    }
                }
                if (config.driver.axes != null) {
                    driverAxisMap.put(
                        config.driver.axes.getOrDefault("leftTrigger", "empty"),
                        Controller.Axis.LEFT_TRIGGER
                    );
                    driverAxisMap.put(
                        config.driver.axes.getOrDefault("rightTrigger", "empty"),
                        Controller.Axis.RIGHT_TRIGGER
                    );
                }
                if (config.driver.buttonpad != null) {
                    if (config.driver.buttonpad.x != null) driverButtonMap.put(
                        config.driver.buttonpad.x,
                        Controller.Button.X
                    );
                    if (config.driver.buttonpad.y != null) driverButtonMap.put(
                        config.driver.buttonpad.y,
                        Controller.Button.Y
                    );
                    if (config.driver.buttonpad.a != null) driverButtonMap.put(
                        config.driver.buttonpad.a,
                        Controller.Button.A
                    );
                    if (config.driver.buttonpad.b != null) driverButtonMap.put(
                        config.driver.buttonpad.b,
                        Controller.Button.B
                    );
                }
                if (config.driver.buttons != null) {
                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("leftBumper", "empty"),
                        Controller.Button.LEFT_BUMPER
                    );
                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("rightBumper", "empty"),
                        Controller.Button.RIGHT_BUMPER
                    );

                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("start", "empty"),
                        Controller.Button.START
                    );
                    driverButtonMap.put(
                        config.driver.buttons.getOrDefault("back", "empty"),
                        Controller.Button.BACK
                    );
                }
                if (config.driver.dpad != null) {
                    if (config.driver.dpad.up != null) driverDpadMap.put(
                        config.driver.dpad.up,
                        0
                    );
                    if (config.driver.dpad.right != null) driverDpadMap.put(
                        config.driver.dpad.right,
                        90
                    );
                    if (config.driver.dpad.down != null) driverDpadMap.put(
                        config.driver.dpad.down,
                        180
                    );
                    if (config.driver.dpad.left != null) driverDpadMap.put(
                        config.driver.dpad.up,
                        270
                    );
                }
            }
            if (config.operator != null) {
                if (config.operator.rumble != null) {
                    operatorRumble = config.operator.rumble;
                }
                if (config.operator.joysticks != null) {
                    if (config.operator.joysticks.get("left") != null) {
                        if (config.operator.joysticks.get("left").horizontal != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("left").horizontal,
                                Controller.Axis.LEFT_X
                            );
                        }
                        if (config.operator.joysticks.get("left").vertical != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("left").vertical,
                                Controller.Axis.LEFT_Y
                            );
                        }
                    }
                    if (config.operator.joysticks.get("right") != null) {
                        if (config.operator.joysticks.get("right").horizontal != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("right").horizontal,
                                Controller.Axis.RIGHT_X
                            );
                        }
                        if (config.operator.joysticks.get("right").vertical != null) {
                            operatorAxisMap.put(
                                config.operator.joysticks.get("right").vertical,
                                Controller.Axis.RIGHT_Y
                            );
                        }
                    }
                }
                if (config.operator.axes != null) {
                    operatorAxisMap.put(
                        config.operator.axes.getOrDefault("leftTrigger", "empty"),
                        Controller.Axis.LEFT_TRIGGER
                    );
                    operatorAxisMap.put(
                        config.operator.axes.getOrDefault("rightTrigger", "empty"),
                        Controller.Axis.RIGHT_TRIGGER
                    );
                }
                if (config.operator.buttonpad != null) {
                    if (config.operator.buttonpad.x != null) operatorButtonMap.put(
                        config.operator.buttonpad.x,
                        Controller.Button.X
                    );
                    if (config.operator.buttonpad.y != null) operatorButtonMap.put(
                        config.operator.buttonpad.y,
                        Controller.Button.Y
                    );
                    if (config.operator.buttonpad.a != null) operatorButtonMap.put(
                        config.operator.buttonpad.a,
                        Controller.Button.A
                    );
                    if (config.operator.buttonpad.b != null) operatorButtonMap.put(
                        config.operator.buttonpad.b,
                        Controller.Button.B
                    );
                }
                if (config.operator.buttons != null) {
                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("leftBumper", "empty"),
                        Controller.Button.LEFT_BUMPER
                    );
                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("rightBumper", "empty"),
                        Controller.Button.RIGHT_BUMPER
                    );

                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("start", "empty"),
                        Controller.Button.START
                    );
                    operatorButtonMap.put(
                        config.operator.buttons.getOrDefault("back", "empty"),
                        Controller.Button.BACK
                    );
                }
                if (config.operator.dpad != null) {
                    if (config.operator.dpad.up != null) operatorDpadMap.put(
                        config.operator.dpad.up,
                        0
                    );
                    if (config.operator.dpad.right != null) operatorDpadMap.put(
                        config.operator.dpad.right,
                        90
                    );
                    if (config.operator.dpad.down != null) operatorDpadMap.put(
                        config.operator.dpad.down,
                        180
                    );
                    if (config.operator.dpad.left != null) operatorDpadMap.put(
                        config.operator.dpad.left,
                        270
                    );
                }
            }
            if (config.mrButtons != null){
                if(config.mrButtons.rumble != null){
                    mrButtonsRumble = config.mrButtons.rumble;
                }
                if(config.mrButtons.joysticks != null){
                    if (config.mrButtons.joysticks.get("left") != null) {
                        if (config.mrButtons.joysticks.get("left").horizontal != null) {
                            mrButtonAxisMap.put(
                                config.mrButtons.joysticks.get("left").horizontal,
                                Controller.Axis.LEFT_X
                            );
                        }
                        if (config.mrButtons.joysticks.get("left").vertical != null) {
                            mrButtonAxisMap.put(
                                config.mrButtons.joysticks.get("left").vertical,
                                Controller.Axis.LEFT_Y
                            );
                        }
                    }
                    if (config.mrButtons.joysticks.get("right") != null) {
                        if (config.mrButtons.joysticks.get("right").horizontal != null) {
                            mrButtonAxisMap.put(
                                config.mrButtons.joysticks.get("right").horizontal,
                                Controller.Axis.RIGHT_X
                            );
                        }
                        if (config.mrButtons.joysticks.get("right").vertical != null) {
                            mrButtonAxisMap.put(
                                config.mrButtons.joysticks.get("right").vertical,
                                Controller.Axis.RIGHT_Y
                            );
                        }
                    }
                }
                if (config.mrButtons.axes != null) {
                    mrButtonAxisMap.put(
                        config.mrButtons.axes.getOrDefault("leftTrigger", "empty"),
                        Controller.Axis.LEFT_TRIGGER
                    );
                    mrButtonAxisMap.put(
                        config.operator.axes.getOrDefault("rightTrigger", "empty"),
                        Controller.Axis.RIGHT_TRIGGER
                    );
                }
                if (config.mrButtons.buttonpad != null) {
                    if (config.mrButtons.buttonpad.x != null) mrButtons_ButtonMap.put(
                        config.mrButtons.buttonpad.x,
                        Controller.Button.X
                    );
                    if (config.mrButtons.buttonpad.y != null) mrButtons_ButtonMap.put(
                        config.mrButtons.buttonpad.y,
                        Controller.Button.Y
                    );
                    if (config.mrButtons.buttonpad.a != null) mrButtons_ButtonMap.put(
                        config.mrButtons.buttonpad.a,
                        Controller.Button.A
                    );
                    if (config.mrButtons.buttonpad.b != null) mrButtons_ButtonMap.put(
                        config.mrButtons.buttonpad.b,
                        Controller.Button.B
                    );
                }
                if (config.mrButtons.buttons != null) {
                    mrButtons_ButtonMap.put(
                        config.mrButtons.buttons.getOrDefault("leftBumper", "empty"),
                        Controller.Button.LEFT_BUMPER
                    );
                    mrButtons_ButtonMap.put(
                        config.mrButtons.buttons.getOrDefault("rightBumper", "empty"),
                        Controller.Button.RIGHT_BUMPER
                    );

                    mrButtons_ButtonMap.put(
                        config.mrButtons.buttons.getOrDefault("start", "empty"),
                        Controller.Button.START
                    );
                    mrButtons_ButtonMap.put(
                        config.mrButtons.buttons.getOrDefault("back", "empty"),
                        Controller.Button.BACK
                    );
                }
                if (config.mrButtons.dpad != null) {
                    if (config.mrButtons.dpad.up != null) mrButtonsDpadMap.put(
                        config.mrButtons.dpad.up,
                        0
                    );
                    if (config.mrButtons.dpad.right != null) mrButtonsDpadMap.put(
                        config.mrButtons.dpad.right,
                        90
                    );
                    if (config.mrButtons.dpad.down != null) mrButtonsDpadMap.put(
                        config.mrButtons.dpad.down,
                        180
                    );
                    if (config.mrButtons.dpad.left != null) mrButtonsDpadMap.put(
                        config.mrButtons.dpad.left,
                        270
                    );
                }
            }

        }
    }

    /**
     * Returns driver-side buttons
     * @return
     */
    public HashMap<String, Controller.Button> getDriverButtonMap() {
        return driverButtonMap;
    }

    /**
     * Returns driver-side axes
     * @return
     */
    public HashMap<String, Controller.Axis> getDriverAxisMap() {
        return driverAxisMap;
    }

    /**
     * Returns driver-side dpad
     * @return
     */
    public HashMap<String, Integer> getDriverDpadMap() {
        return driverDpadMap;
    }

    /**
     * A simple hashMap checking utility for all controls
     * @param key
     * @return
     */
    public boolean driverMapContainsKey(String key) {
        return (
            driverAxisMap.containsKey(key) ||
            driverButtonMap.containsKey(key) ||
            driverDpadMap.containsKey(key)
        );
    }

    /**
     * Return operator-side buttons
     * @return
     */
    public HashMap<String, Controller.Button> getOperatorButtonMap() {
        return operatorButtonMap;
    }

    /**
     * Return operator-side axes
     * @return
     */
    public HashMap<String, Controller.Axis> getOperatorAxisMap() {
        return operatorAxisMap;
    }

    /**
     * Returns operator-side dpad
     * @return
     */
    public HashMap<String, Integer> getOperatorDpadMap() {
        return operatorDpadMap;
    }

    /**
     * A simple hashMap checking utility for all controls
     * @param key
     * @return
     */
    public boolean operatorMapContainsKey(String key) {
        return (
            operatorAxisMap.containsKey(key) ||
            operatorButtonMap.containsKey(key) ||
            operatorDpadMap.containsKey(key)
        );
    }

    public HashMap<String, Controller.Button> getMrButtons_ButtonMap() {
        return mrButtons_ButtonMap;
    }

    public HashMap<String, Controller.Axis> getMrButtonAxisMap() {
        return mrButtonAxisMap;
    }

    public HashMap<String, Integer> getMrButtonsDpadMap() {
        return mrButtonsDpadMap;
    }

    public boolean mrButtonsMapContrainsKey(String key){
        return (mrButtonAxisMap.containsKey(key) ||
                mrButtons_ButtonMap.containsKey(key) ||
                mrButtonsDpadMap.containsKey(key));
    }


}
