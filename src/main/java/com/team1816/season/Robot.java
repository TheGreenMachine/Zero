package com.team1816.season;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.controlboard.ActionManager;
import com.team1816.lib.controlboard.ControlBoard;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.SubsystemLooper;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.auto.commands.AlignElevatorCommand;
import com.team1816.season.auto.commands.AutoScoreCommand;
import com.team1816.season.auto.commands.TargetAlignCommand;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.DrivetrainTargets;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;

import static com.team1816.lib.controlboard.ControlUtils.createAction;
import static com.team1816.lib.controlboard.ControlUtils.createHoldAction;

public class Robot extends TimedRobot {

    /**
     * Looper
     */
    private final Looper enabledLoop;
    private final Looper disabledLoop;

    /**
     * Controls
     */
    private IControlBoard controlBoard;
    private ActionManager actionManager;

    private final Infrastructure infrastructure;
    private final SubsystemLooper subsystemManager;

    /**
     * State Managers
     */
    private final Orchestrator orchestrator;
    private final RobotState robotState;

    /**
     * Subsystems
     */
    private final Drive drive;

    private final LedManager ledManager;
    private final Camera camera;
    private final Elevator elevator;
    private final Collector collector;

    private DigitalInput zeroingButton;
    private Boolean zeroing = false;
    private boolean lastButton;

    /**
     * Factory
     */
    private static RobotFactory factory;

    /**
     * Autonomous
     */
    private final AutoModeManager autoModeManager;
    private Thread autoTargetThread;

    private Thread alignElevatorThread;
    private Thread autoScoreThread;

    private Thread autoTargetAlignThread;

    /**
     * Timing
     */
    private double loopStart;
    public static double looperDt;
    public static double robotDt;
    public static double autoStart;
    public static double teleopStart;

    private DoubleLogEntry robotLoopLogger;
    private DoubleLogEntry looperLogger;

    /**
     * Properties
     */
    private boolean faulted;
    private int grid = 0;
    private int node = 0;
    private Elevator.EXTENSION_STATE level = Elevator.EXTENSION_STATE.MIN;

    private boolean desireCube = true;

    public static boolean runningAutoTarget = false;
    public static boolean runningAutoTargetAlign = false;

    public static boolean runningAutoAlign = false;
    public static boolean runningAutoScore = false;
    public static boolean runningAutoBalance = false;
    public Elevator.ANGLE_STATE prevAngleState;
    private double dPadMoveSpeed;

    private boolean snappingToHumanPlayer;
    private boolean snappingToDriver;

    /**
     * Instantiates the Robot by injecting all systems and creating the enabled and disabled loopers
     */
    Robot() {
        super();
        // initialize injector
        Injector.registerModule(new SeasonModule());
        enabledLoop = new Looper(this);
        disabledLoop = new Looper(this);
        drive = (Injector.get(Drive.Factory.class)).getInstance();
        elevator = Injector.get(Elevator.class);
        collector = Injector.get(Collector.class);
        ledManager = Injector.get(LedManager.class);
        camera = Injector.get(Camera.class);
        robotState = Injector.get(RobotState.class);
        orchestrator = Injector.get(Orchestrator.class);
        infrastructure = Injector.get(Infrastructure.class);
        subsystemManager = Injector.get(SubsystemLooper.class);
        autoModeManager = Injector.get(AutoModeManager.class);

        prevAngleState = Elevator.ANGLE_STATE.STOW;
        if (RobotBase.isReal()) {
            zeroingButton = new DigitalInput((int) factory.getConstant("zeroingButton", -1));
        }
        if (Constants.kLoggingRobot) {
            robotLoopLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/Robot");
            looperLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/RobotState");
        }

        dPadMoveSpeed = factory.getConstant(Drive.NAME, "dPadMoveSpeed", 0);
    }

    /**
     * Returns the static factory instance of the Robot
     *
     * @return RobotFactory
     */
    public static RobotFactory getFactory() {
        if (factory == null) factory = Injector.get(RobotFactory.class);
        return factory;
    }

    /**
     * Returns the length of the last loop that the Robot was on
     *
     * @return duration (ms)
     */
    public Double getLastRobotLoop() {
        return (Timer.getFPGATimestamp() - loopStart) * 1000;
    }

    /**
     * Returns the duration of the last enabled loop
     *
     * @return duration (ms)
     * @see Looper#getLastLoop()
     */
    public Double getLastSubsystemLoop() {
        return enabledLoop.isRunning() ? enabledLoop.getLastLoop() : disabledLoop.getLastLoop();
    }

    /**
     * Actions to perform when the robot has just begun being powered and is done booting up.
     * Initializes the robot by injecting the controlboard, and registering all subsystems.
     */
    @Override
    public void robotInit() {
        try {
            /** Register All Subsystems */
            controlBoard = Injector.get(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            // Remember to register our elevator and collector subsystems below!! The subsystem manager deals with calling
            // read/writetohardware on a loop, but it can only call read/write if it recognizes said subsystem. To recognize
            // your subsystem, just add it alongside the drive, ledManager, and camera parameters :)
            subsystemManager.setSubsystems(drive, ledManager, camera, elevator, collector);

            /** Logging */
            if (Constants.kLoggingRobot) {
                var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
                var robotName = System.getenv("ROBOT_NAME");
                if (robotName == null) robotName = "default";
                var logFileDir = "/home/lvuser/";
                // if there is a USB drive use it
                if (Files.exists(Path.of("/media/sda1"))) {
                    logFileDir = "/media/sda1/";
                }
                if (RobotBase.isSimulation()) {
                    if (System.getProperty("os.name").toLowerCase().contains("win")) {
                        logFileDir = System.getenv("temp") + "\\";
                    } else {
                        logFileDir = System.getProperty("user.dir") + "/";
                    }
                }
                var filePath = logFileDir + robotName + "_" + logFile + ".bag";
                DataLogManager.start();
                DriverStation.startDataLog(DataLogManager.getLog(), false);
            }

            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);
            // zeroing ypr - (-90) b/c our pigeon is mounted with the "y" axis facing forward
            infrastructure.resetPigeon(Rotation2d.fromDegrees(-90));
            subsystemManager.zeroSensors();
            faulted = true; // elevator not zeroed on bootup - letting ppl know

            /** Register ControlBoard */
            controlBoard = Injector.get(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    createAction(
                        () -> controlBoard.getAsBool("zeroPose"),
                        () -> {
                            drive.zeroSensors(robotState.allianceColor == Color.BLUE ? Constants.kDefaultZeroingPose : Constants.kFlippedZeroingPose);
                        }
                    ),
//                    createAction(
//                        () -> controlBoard.getAsBool("autoTarget"),
//                        () -> {
//                            if (robotState.allianceColor == Color.BLUE) {
//                                robotState.target = DrivetrainTargets.blueTargets.get((2 - grid) * 3 + (2 - node));
//                            } else {
//                                robotState.target = DrivetrainTargets.redTargets.get(grid * 3 + node);
//                            }
//                            if (!runningAutoTarget) {
//                                runningAutoTarget = true;
//                                orchestrator.updatePoseWithCamera();
//                                double distance = robotState.fieldToVehicle.getTranslation().getDistance(robotState.target.getTranslation());
//                                if (distance < Constants.kMinTrajectoryDistance) {
//                                    GreenLogger.log("Distance to target is " + distance + " m");
//                                    GreenLogger.log("Too close to target! can not start trajectory!");
//                                } else {
//                                    GreenLogger.log("Drive trajectory action started!");
//                                    TargetTrajectoryCommand command = new TargetTrajectoryCommand();
//                                    autoTargetThread = new Thread(command::run);
//                                    ledManager.indicateStatus(LedManager.RobotStatus.AUTONOMOUS, LedManager.ControlState.FAST_BLINK);
//                                    autoTargetThread.start();
//                                }
//                            } else {
//                                autoTargetThread.stop();
//                                ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.SOLID);
//                                GreenLogger.log("Stopped! driving to trajectory canceled!");
//                                runningAutoTarget = !runningAutoTarget;
//                            }
//                        }
//                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoTargetAlign"),
                        () -> {
                            if (robotState.allianceColor == Color.BLUE) {
                                robotState.target = DrivetrainTargets.blueTargets.get((2 - grid) * 3 + (2 - node));
                            } else {
                                robotState.target = DrivetrainTargets.redTargets.get(grid * 3 + node);
                            }
                            if (!runningAutoTargetAlign) {
                                runningAutoTargetAlign = true;
                                orchestrator.updatePoseWithCamera();
                                double distance = robotState.fieldToVehicle.getTranslation().getDistance(robotState.target.getTranslation());
                                if (distance < Constants.kMinTrajectoryDistance) {
                                    GreenLogger.log("Distance to target is " + distance + " m");
                                    GreenLogger.log("Too close to target! can not start trajectory! setting elevator extension to: " + level.name());
                                    AlignElevatorCommand command = new AlignElevatorCommand(level);
                                    autoTargetAlignThread = new Thread(command::run);
                                } else {
                                    GreenLogger.log("Drive trajectory action started!");
                                    TargetAlignCommand command = new TargetAlignCommand(level);
                                    autoTargetAlignThread = new Thread(command::run);
                                }
                                ledManager.indicateStatus(LedManager.RobotStatus.AUTONOMOUS, LedManager.ControlState.SOLID);
                                autoTargetAlignThread.start();
                            } else {
                                autoTargetAlignThread.stop();
                                GreenLogger.log("Stopped! driving to trajectory canceled!");
                                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED, LedManager.ControlState.SOLID);
                                runningAutoTargetAlign = !runningAutoTargetAlign;
                            }
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("brakeMode"),
                        drive::setBraking
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("slowMode"),
                        () -> {
                            drive.setSlowMode(!drive.getSlowMode());
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("midSlowMode"),
                        () -> {
                            drive.setMidSlowMode(!drive.getMidSlowMode());
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoBalance"),
                        () -> {
                            drive.setAutoBalance(!drive.isAutoBalancing());
                        }
                    ),
                    /*createHoldAction(
                        () -> controlBoard.getAsBool("autoBalance"),
                        (pressed) -> {
                            if (pressed) {
                                drive.setAutoBalance(true);
                                ledManager.indicateStatus(LedManager.RobotStatus.BALANCE, LedManager.ControlState.BLINK);
                            } else {
                                drive.setAutoBalance(false);
                                ledManager.indicateStatus(LedManager.RobotStatus.BALANCE, LedManager.ControlState.SOLID);
                            }
                        }
                    ),*/
                    createHoldAction(
                        () -> controlBoard.getAsBool("intakeCone"),
                        (pressed) -> {
                            if (pressed) {
                                if (
                                    elevator.getDesiredAngleState() == Elevator.ANGLE_STATE.SHELF_COLLECT
                                ) { // collects from shelf
                                    collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.SHELF);
                                } else { // collects from floor
                                    collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.FLOOR);
                                }
                                drive.setMidSlowMode(true);
                                ledManager.indicateStatus(LedManager.RobotStatus.CONE, LedManager.ControlState.BLINK); // indicates on LEDs
                            } else {
                                collector.setDesiredState(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW);
                                drive.setMidSlowMode(false);
                                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
                            }
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("intakeCube"),
                        (pressed) -> {
                            if (pressed) {
                                if (
                                    elevator.getDesiredAngleState() == Elevator.ANGLE_STATE.SHELF_COLLECT
                                ) { // collects from shelf
                                    collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CUBE, Collector.PIVOT_STATE.SHELF);
                                } else { // collects from floor
                                    collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CUBE, Collector.PIVOT_STATE.FLOOR);
                                }
                                drive.setMidSlowMode(true);
                                ledManager.indicateStatus(LedManager.RobotStatus.CUBE, LedManager.ControlState.BLINK); // indicates on LEDs
                            } else {
                                collector.setDesiredState(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW);
                                drive.setMidSlowMode(false);
                                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("toggleArmScoreCollect"),
                        () -> {
                            if (elevator.getDesiredAngleState() != Elevator.ANGLE_STATE.STOW) {
                                elevator.setDesiredState(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN);
                            } else {
                                elevator.setDesiredState(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN);
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("snapToHumanPlayer"),
                        () -> {
                            snappingToHumanPlayer = !snappingToHumanPlayer;
                            snappingToDriver = false;
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("snapToDriver"),
                        () -> {
                            snappingToDriver = !snappingToDriver;
                            snappingToHumanPlayer = false;
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("shelfPos"),
                        (pressed) -> {
                            elevator.setDesiredState(Elevator.ANGLE_STATE.SHELF_COLLECT, Elevator.EXTENSION_STATE.SHELF_COLLECT);
                        }
                    ),
                    // Operator Gamepad
                    createAction(
                        () -> controlBoard.getAsBool("updatePoseWithCamera"),
                        orchestrator::updatePoseWithCamera
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("outtake"),
                        (pressed) -> {
                            collector.outtakeGamePiece(pressed);
                            if (pressed) {
                                if (robotState.actualGameElement == Collector.GAME_ELEMENT.CONE) {
                                    ledManager.indicateStatus(LedManager.RobotStatus.CONE, LedManager.ControlState.BLINK);
                                } else if (robotState.actualGameElement == Collector.GAME_ELEMENT.CUBE) {
                                    ledManager.indicateStatus(LedManager.RobotStatus.CUBE, LedManager.ControlState.BLINK);
                                }
                            } else {
                                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED, LedManager.ControlState.SOLID);
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("extendStage"),
                        () -> {
                            Elevator.EXTENSION_STATE extensionState = elevator.getDesiredExtensionState();

                            if (extensionState == Elevator.EXTENSION_STATE.MIN) {
                                elevator.setDesiredExtensionState(Elevator.EXTENSION_STATE.MID);
                            } else if (extensionState == Elevator.EXTENSION_STATE.MID) {
                                elevator.setDesiredExtensionState(Elevator.EXTENSION_STATE.MAX);
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("descendStage"),
                        () -> {
                            Elevator.EXTENSION_STATE extensionState = elevator.getDesiredExtensionState();

                            if (extensionState == Elevator.EXTENSION_STATE.MID) {
                                elevator.setDesiredExtensionState(Elevator.EXTENSION_STATE.MIN);
                            } else if (extensionState == Elevator.EXTENSION_STATE.MAX) {
                                elevator.setDesiredExtensionState(Elevator.EXTENSION_STATE.MID);
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("armStow"),
                        () -> {
                            elevator.setDesiredState(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN);
                            collector.setDesiredState(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW);
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("armCollect"),
                        () -> {
                            elevator.setDesiredAngleState(Elevator.ANGLE_STATE.COLLECT);
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("extendMin"),
                        () -> {
                            GreenLogger.log("extend min");
                            if (!runningAutoAlign) {
                                runningAutoAlign = true;
                                GreenLogger.log("Auto align action started!");
                                AlignElevatorCommand command = new AlignElevatorCommand(Elevator.EXTENSION_STATE.MIN);
                                alignElevatorThread = new Thread(command::run);
                                ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.BLINK);
                                alignElevatorThread.start();
                            } else {
                                alignElevatorThread.stop();
                                elevator.setDesiredState(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN);
                                GreenLogger.log("Stopped! Auto align cancelled!");
                                runningAutoAlign = !runningAutoAlign;
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("extendMid"),
                        () -> {
                            GreenLogger.log("extend mid");
                            if (!runningAutoAlign) {
                                runningAutoAlign = true;
                                GreenLogger.log("Auto align action started!");
                                AlignElevatorCommand command = new AlignElevatorCommand(Elevator.EXTENSION_STATE.MID);
                                alignElevatorThread = new Thread(command::run);
                                ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.BLINK);
                                alignElevatorThread.start();
                            } else {
                                alignElevatorThread.stop();
                                elevator.setDesiredState(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MID);
                                GreenLogger.log("Stopped! Auto align cancelled!");
                                runningAutoAlign = !runningAutoAlign;
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("extendMax"),
                        () -> {
                            GreenLogger.log("extend max");
                            if (!runningAutoAlign) {
                                runningAutoAlign = true;
                                GreenLogger.log("Auto align action started!");
                                AlignElevatorCommand command = new AlignElevatorCommand(Elevator.EXTENSION_STATE.MAX);
                                alignElevatorThread = new Thread(command::run);
                                ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.BLINK);
                                alignElevatorThread.start();
                            } else {
                                alignElevatorThread.stop();
                                elevator.setDesiredState(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MAX);
                                GreenLogger.log("Stopped! Auto align cancelled!");
                                runningAutoAlign = !runningAutoAlign;
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoScore"),
                        () -> {
                            GreenLogger.log("autoscore");
                            if (!runningAutoScore) {
                                runningAutoScore = true;
                                GreenLogger.log("Auto Score action started!");
                                AutoScoreCommand command = new AutoScoreCommand(collector.getCurrentGameElement(), elevator.getDesiredExtensionState());
                                autoScoreThread = new Thread(command::run);
                                ledManager.indicateStatus(LedManager.RobotStatus.ON_TARGET, LedManager.ControlState.BLINK);
                                autoScoreThread.start();
                            } else {
                                autoScoreThread.stop();
                                GreenLogger.log("Stopped! Auto scoring cancelled!");
                                runningAutoScore = !runningAutoScore;
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("toggleCollectorPivot"),
                        () -> {
                            if (robotState.actualElevatorAngleState == Elevator.ANGLE_STATE.SCORE) {
                                if (collector.getDesiredPivotState() == Collector.PIVOT_STATE.SCORE) {
                                    collector.setDesiredState(collector.getDesiredRollerState(), Collector.PIVOT_STATE.STOW);
                                } else if (collector.getDesiredPivotState() == Collector.PIVOT_STATE.STOW) {
                                    collector.setDesiredState(collector.getDesiredRollerState(), Collector.PIVOT_STATE.SCORE);
                                }
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("grid1"),
                        () -> {
                            grid = 0;
                            GreenLogger.log("Grid changed to 0");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("grid2"),
                        () -> {
                            grid = 1;
                            GreenLogger.log("Grid changed to 1");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("grid3"),
                        () -> {
                            grid = 2;
                            GreenLogger.log("Grid changed to 2");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("node1"),
                        () -> {
                            node = 0;
                            GreenLogger.log("Node changed to 0");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("node2"),
                        () -> {
                            node = 1;
                            GreenLogger.log("Node changed to 1");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("node3"),
                        () -> {
                            node = 2;
                            GreenLogger.log("Node changed to 2");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("level1"),
                        () -> {
                            level = Elevator.EXTENSION_STATE.MIN;
                            GreenLogger.log("Score level changed to Low");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("level2"),
                        () -> {
                            level = Elevator.EXTENSION_STATE.MID;
                            GreenLogger.log("Score level changed to Mid");
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("level3"),
                        () -> {
                            level = Elevator.EXTENSION_STATE.MAX;
                            GreenLogger.log("Score level changed to High");
                        }
                    )
                );
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the disabled period
     */
    @Override
    public void disabledInit() {
        try {
            if (autoTargetThread != null && autoTargetThread.isAlive()) {
                autoTargetThread.stop();
            }
            if (autoScoreThread != null && autoScoreThread.isAlive()) {
                autoScoreThread.stop();
            }
            if (alignElevatorThread != null && alignElevatorThread.isAlive()) {
                alignElevatorThread.stop();
            }
            if (autoTargetAlignThread != null && autoTargetAlignThread.isAlive()) {
                autoTargetAlignThread.stop();
            }

            enabledLoop.stop();
            // Stop any running autos
            autoModeManager.stopAuto();
            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);
            ledManager.writeToHardware();

            if (autoModeManager.getSelectedAuto() == null) {
                autoModeManager.reset();
            }

            subsystemManager.stop();

            robotState.resetAllStates();
            drive.zeroSensors();

            disabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the autonomous period
     */
    @Override
    public void autonomousInit() {
        disabledLoop.stop();
        ledManager.setDefaultStatus(LedManager.RobotStatus.AUTONOMOUS);
        ledManager.indicateStatus(LedManager.RobotStatus.AUTONOMOUS);

        drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());

        drive.setControlState(Drive.ControlState.TRAJECTORY_FOLLOWING);
        autoModeManager.startAuto();

        autoStart = Timer.getFPGATimestamp();
        enabledLoop.start();
    }

    /**
     * Actions to perform when the robot has entered the teleoperated period
     */
    @Override
    public void teleopInit() {
        try {
            disabledLoop.stop();
            ledManager.setDefaultStatus(LedManager.RobotStatus.ENABLED);
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);

            infrastructure.startCompressor();
            elevator.setDesiredState(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN);

            teleopStart = Timer.getFPGATimestamp();
            enabledLoop.start();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform when the robot has entered the test period
     */
    @Override
    public void testInit() {
        try {
            double initTime = System.currentTimeMillis();

            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED, LedManager.ControlState.BLINK);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writeToHardware();
            }

            enabledLoop.stop();
            disabledLoop.start();
            drive.zeroSensors();

            ledManager.indicateStatus(LedManager.RobotStatus.DISABLED, LedManager.ControlState.BLINK);

            if (subsystemManager.testSubsystems()) {
                GreenLogger.log("ALL SYSTEMS PASSED");
                ledManager.indicateStatus(LedManager.RobotStatus.ENABLED);
            } else {
                System.err.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
                ledManager.indicateStatus(LedManager.RobotStatus.ERROR);
            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform periodically on the robot when the robot is powered
     */
    @Override
    public void robotPeriodic() {
        try {
            // updating loop timers
            Robot.looperDt = getLastSubsystemLoop();
            Robot.robotDt = getLastRobotLoop();
            loopStart = Timer.getFPGATimestamp();

            if (Constants.kLoggingRobot) {
                looperLogger.append(looperDt);
                robotLoopLogger.append(robotDt);
            }

            subsystemManager.outputToSmartDashboard(); // update shuffleboard for subsystem values
            robotState.outputToSmartDashboard(); // update robot state on field for Field2D widget
            autoModeManager.outputToSmartDashboard(); // update shuffleboard selected auto mode
        } catch (Throwable t) {
            faulted = true;
            GreenLogger.log(t.getMessage());
        }
    }

    /**
     * Actions to perform periodically when the robot is in the disabled period
     */
    @Override
    public void disabledPeriodic() {
        try {
            if (RobotController.getUserButton()) {
                drive.zeroSensors(Constants.kDefaultZeroingPose);
                ledManager.indicateStatus(LedManager.RobotStatus.CUBE);
            } else {
                // non-camera LEDs will flash red if robot periodic updates fail
                if (faulted) {
                    if (ledManager.getCurrentControlStatus() != LedManager.RobotStatus.ERROR) {
                        ledManager.indicateStatus(LedManager.RobotStatus.ERROR, LedManager.ControlState.BLINK);
                    }
                    ledManager.writeToHardware();
                }
            }

            if (RobotBase.isReal()) {
                // logic for zeroing elevator
                if (lastButton != zeroingButton.get() && lastButton) { // will only be true when changing from false to true
                    if (zeroing == null) { // zeroing
                        faulted = false;
                        zeroing = true;
                        elevator.zeroSensors();
                        collector.zeroSensors();
                        ledManager.indicateStatus(LedManager.RobotStatus.ZEROING, LedManager.ControlState.BLINK);
                        ledManager.writeToHardware();
                        infrastructure.resetPigeon(Rotation2d.fromDegrees(-90));
                    } else if (zeroing) { // ready
                        zeroing = false;
                        elevator.setBraking(true);
                        collector.setBraking(true);
                        ledManager.indicateStatus(LedManager.RobotStatus.DISABLED, LedManager.ControlState.SOLID);
                        ledManager.writeToHardware();
                    } else { // needs zeroing
                        zeroing = null;
                        elevator.setBraking(false);
                        collector.setBraking(false);
                        ledManager.indicateStatus(LedManager.RobotStatus.ERROR, LedManager.ControlState.BLINK);
                        ledManager.writeToHardware();

                        faulted = true;
                    }
                }
                lastButton = zeroingButton.get();

                if (ledManager.getCurrentControlStatus() == LedManager.RobotStatus.ZEROING) {
                    // only keep looping through write if zeroing elevator cus we need to update its blinking
                    ledManager.writeToHardware();
                }
            }

            // Periodically check if drivers changed desired auto - if yes, then update the robot's position on the field
            if (autoModeManager.update()) {
                drive.zeroSensors(autoModeManager.getSelectedAuto().getInitialPose());
                robotState.field
                    .getObject("Trajectory")
                    .setTrajectory(
                        autoModeManager.getSelectedAuto().getCurrentTrajectory()
                    );
            }

            if (drive.isDemoMode()) { // Demo-mode
                drive.update();
            }

        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Actions to perform periodically when the robot is in the autonomous period
     */
    @Override
    public void autonomousPeriodic() {
        robotState.field
            .getObject("Trajectory")
            .setTrajectory(autoModeManager.getSelectedAuto().getCurrentTrajectory());
    }

    /**
     * Actions to perform periodically when the robot is in the teleoperated period
     */
    @Override
    public void teleopPeriodic() {

        try {
            manualControl();
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
    }

    /**
     * Sets manual inputs for subsystems like the drivetrain when criteria met
     */
    public void manualControl() {
        actionManager.update();

        if (drive.isAutoBalancing()) {
            ChassisSpeeds fieldRelativeChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                -controlBoard.getAsDouble("strafe"),
                0,
                robotState.driverRelativeFieldToVehicle.getRotation());
            drive.autoBalance(fieldRelativeChassisSpeed);

        } else if (snappingToDriver || snappingToHumanPlayer) { // dpad bang-bang controller for fine alignment
            GreenLogger.log(snappingToDriver ? snappingToDriver : snappingToHumanPlayer);
            double rotation = 0;
            if (snappingToDriver) { //down on the d pad, snap to driver
                double rotVal = MathUtil.inputModulus(
                    robotState.driverRelativeFieldToVehicle.getRotation().getDegrees(), robotState.allianceColor == Color.BLUE ? -180 : 180, robotState.allianceColor == Color.BLUE ? 180 : -180
                );
                if ((rotVal < 2 && rotVal > -2) || (rotVal < -135 || rotVal > 135)) {
                    rotation = 0;
                } else {
                    rotation = 90 / rotVal;
                }
            } else if (snappingToHumanPlayer) { //up on the d pad, snap to human player
                double rotVal = MathUtil.inputModulus(
                    robotState.driverRelativeFieldToVehicle.getRotation().getDegrees(), robotState.allianceColor == Color.BLUE ? -180 : 180, robotState.allianceColor == Color.BLUE ? 180 : -180
                );
                if ((rotVal < 45 && rotVal > -45) || (rotVal < -178 || rotVal > 178)) {
                    rotation = 0;
                } else {
                    rotation = 90 / rotVal;
                }
            }
            SwerveModuleState[] dPadDrivingStates = SwerveDrive.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, rotation, robotState.driverRelativeFieldToVehicle.getRotation())
            );

            if(rotation == 0){
                Rotation2d heading = Rotation2d.fromDegrees(90).minus(robotState.driverRelativeFieldToVehicle.getRotation());
                SwerveModuleState templateState = new SwerveModuleState(0, heading);
                SwerveModuleState[] statePassIn = new SwerveModuleState[]{templateState, templateState, templateState, templateState};
                ((SwerveDrive) drive).setModuleStates(statePassIn);
                snappingToDriver = false;
                snappingToHumanPlayer = false;
            } else {
                ((SwerveDrive) drive).setModuleStates(dPadDrivingStates);
            }

        } else {
            drive.setTeleopInputs(
                -controlBoard.getAsDouble("throttle"),
                -controlBoard.getAsDouble("strafe"),
                controlBoard.getAsDouble("rotation")
            );
        }
    }

    /**
     * Actions to perform periodically when the robot is in the test period
     */
    @Override
    public void testPeriodic() {
    }
}
