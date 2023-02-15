package com.team1816.season;

import badlog.lib.BadLog;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.controlboard.ActionManager;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.SubsystemLooper;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.DrivetrainLogger;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.auto.modes.AutoBalanceMode;
import com.team1816.season.auto.modes.TrajectoryToTargetMode;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
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
     * Logger
     */
    private static BadLog logger;

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

    /**
     * Factory
     */
    private static RobotFactory factory;

    /**
     * Autonomous
     */
    private final AutoModeManager autoModeManager;
    private Thread autoTargetThread;
    private Thread autoBalanceThread;

    /**
     * Timing
     */
    private double loopStart;
    public static double dt;
    public static double autoStart;
    public static double teleopStart;

    /**
     * Properties
     */
    private boolean faulted;
    public static boolean runningAutoTarget = false;
    public static boolean runningAutoBalance = false;

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
    public Double getLastEnabledLoop() {
        dt = enabledLoop.getLastLoop();
        return enabledLoop.getLastLoop();
    }

    /**
     * Actions to perform when the robot has just begun being powered and is done booting up.
     * Initializes the robot by injecting the controlboard, registering all subsystems, and setting up BadLogs.
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

            /** Register BadLogs */
            if (Constants.kIsBadlogEnabled) {
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
                logger = BadLog.init(filePath);

                BadLog.createTopic(
                    "Timings/Looper",
                    "ms",
                    this::getLastEnabledLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/RobotLoop",
                    "ms",
                    this::getLastRobotLoop,
                    "hide",
                    "join:Timings"
                );
                BadLog.createTopic(
                    "Timings/Timestamp",
                    "s",
                    Timer::getFPGATimestamp,
                    "xaxis",
                    "hide"
                );
                BadLog.createTopic(
                    "Vision/Distance",
                    "inches",
                    robotState::getDistanceToGoal
                );
                BadLog.createValue("Drivetrain PID", drive.pidToString());
                DrivetrainLogger.init(drive);
                BadLog.createTopic(
                    "PDP/Current",
                    "Amps",
                    infrastructure.getPd()::getTotalCurrent
                );

                BadLog.createTopic(
                    "Pigeon/Yaw",
                    "degrees",
                    infrastructure::getYaw
                );
                BadLog.createTopic(
                    "Pigeon/Pitch",
                    "degrees",
                    infrastructure::getPitch
                );
                BadLog.createTopic(
                    "Pigeon/Roll",
                    "degrees",
                    infrastructure::getRoll
                );
                logger.finishInitialization();
            }
            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);
            subsystemManager.zeroSensors();

            /** Register ControlBoard */
            controlBoard = Injector.get(IControlBoard.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            actionManager =
                new ActionManager(
                    // Driver Gamepad
                    createAction(
                        () -> controlBoard.getAsBool("zeroPose"),
                        () -> {
                            drive.zeroSensors(Constants.kDefaultZeroingPose);
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoTarget"),
                        () -> {
                            if (!runningAutoTarget) {
                                runningAutoTarget = true;
                                orchestrator.updatePoseWithCamera();
                                double distance = robotState.fieldToVehicle.getTranslation().getDistance(robotState.target.getTranslation());
                                if (distance < Constants.kMinTrajectoryDistance) {
                                    System.out.println("Distance to target is " + distance + " m");
                                    System.out.println("Too close to target! can not start trajectory!");
                                } else {
                                    System.out.println("Drive trajectory action started!");
                                    TrajectoryToTargetMode mode = new TrajectoryToTargetMode();
                                    autoTargetThread = new Thread(mode::run);
                                    autoTargetThread.start();
                                    System.out.println("Trajectory ended");
                                }
                            } else {
                                autoTargetThread.stop();
                                System.out.println("Stopped! driving to trajectory canceled!");
                                runningAutoTarget = !runningAutoTarget;
                            }
                        }
                    ),
                    createAction(
                        () -> controlBoard.getAsBool("autoBalance"),
                        () -> {
                            if (!runningAutoBalance) {
                                runningAutoBalance = true;
                                System.out.println("Starting auto balance");
                                AutoBalanceMode mode = new AutoBalanceMode();
                                autoBalanceThread = new Thread(mode::run);
                                autoBalanceThread.start();
                                autoBalanceThread = null;
                                System.out.println("Balanced");
                            } else {
                                autoBalanceThread.stop();
                                System.out.println("Stopped! driving to trajectory canceled!");
                                runningAutoBalance = !runningAutoBalance;
                            }
                        }
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("brakeMode"),
                        drive::setBraking
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("slowMode"),
                        drive::setSlowMode
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("intakeCone"),
                        (pressed) -> orchestrator.setCollecting(pressed, false)
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("intakeCube"),
                        (pressed) -> orchestrator.setCollecting(pressed, true)
                    ),
                    createHoldAction(
                        () -> controlBoard.getAsBool("outtake"),
                        orchestrator::setScoring
                    ),
                    // Operator Gamepad
                    createAction( // TODO remove, for testing purposes only
                        () -> controlBoard.getAsBool("updatePose"),
                        orchestrator::updatePoseWithCamera
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
            enabledLoop.stop();

            // Stop any running autos
            autoModeManager.stopAuto();
            ledManager.setDefaultStatus(LedManager.RobotStatus.DISABLED);

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

            infrastructure.startCompressor();

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

            ledManager.blinkStatus(LedManager.RobotStatus.DRIVETRAIN_FLIPPED);
            // Warning - blocks thread - intended behavior?
            while (System.currentTimeMillis() - initTime <= 3000) {
                ledManager.writeToHardware();
            }

            enabledLoop.stop();
            disabledLoop.start();
            drive.zeroSensors();

            ledManager.blinkStatus(LedManager.RobotStatus.DISABLED);

            if (subsystemManager.testSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
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
            subsystemManager.outputToSmartDashboard(); // update shuffleboard for subsystem values
            robotState.outputToSmartDashboard(); // update robot state on field for Field2D widget
            autoModeManager.outputToSmartDashboard(); // update shuffleboard selected auto mode
        } catch (Throwable t) {
            faulted = true;
            System.out.println(t.getMessage());
        }
    }

    /**
     * Actions to perform periodically when the robot is in the disabled period
     */
    @Override
    public void disabledPeriodic() {
        loopStart = Timer.getFPGATimestamp();
        try {
            if (RobotController.getUserButton()) {
                drive.zeroSensors(Constants.kDefaultZeroingPose);
                ledManager.indicateStatus(LedManager.RobotStatus.SEEN_TARGET);
            } else {
                // non-camera LEDs will flash red if robot periodic updates fail
                if (faulted) {
                    ledManager.blinkStatus(LedManager.RobotStatus.ERROR);
                } else {
                    ledManager.indicateStatus(LedManager.RobotStatus.DISABLED);
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
        loopStart = Timer.getFPGATimestamp();
        robotState.field
            .getObject("Trajectory")
            .setTrajectory(autoModeManager.getSelectedAuto().getCurrentTrajectory());

        if (Constants.kIsLoggingAutonomous) {
            logger.updateTopics();
            logger.log();
        }
    }

    /**
     * Actions to perform periodically when the robot is in the teleoperated period
     */
    @Override
    public void teleopPeriodic() {
        loopStart = Timer.getFPGATimestamp();

        try {
            manualControl();
            //            if (orchestrator.needsVisionUpdate() || !robotState.isPoseUpdated) {
            //                robotState.isPoseUpdated = false;
            //                orchestrator.calculatePoseFromCamera();
            //            }
        } catch (Throwable t) {
            faulted = true;
            throw t;
        }
        if (Constants.kIsLoggingTeleOp) {
            logger.updateTopics();
            logger.log();
        }
    }

    /**
     * Sets manual inputs for subsystems like the drivetrain when criteria met
     */
    public void manualControl() {
        actionManager.update();

        drive.setTeleopInputs(
            -controlBoard.getAsDouble("throttle"),
            -controlBoard.getAsDouble("strafe"),
            controlBoard.getAsDouble("rotation")
        );
    }

    /**
     * Actions to perform periodically when the robot is in the test period
     */
    @Override
    public void testPeriodic() {
    }
}
