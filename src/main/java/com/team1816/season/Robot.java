package com.team1816.season;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.input_handler.*;
import com.team1816.lib.input_handler.controlOptions.ActionState;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.SubsystemLooper;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.Util;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.DrivetrainTargets;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Objects;

public class Robot extends TimedRobot {

    /**
     * Looper
     */
    private final Looper enabledLoop;
    private final Looper disabledLoop;

    /**
     * Controls
     */
    private InputHandler inputHandler;

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


    /**
     * Factory
     */
    private static RobotFactory factory;

    /**
     * Autonomous
     */
    private final AutoModeManager autoModeManager;

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

        // TODO: Set up any other subsystems here.

        ledManager = Injector.get(LedManager.class);
        camera = Injector.get(Camera.class);
        robotState = Injector.get(RobotState.class);
        orchestrator = Injector.get(Orchestrator.class);
        infrastructure = Injector.get(Infrastructure.class);
        subsystemManager = Injector.get(SubsystemLooper.class);
        autoModeManager = Injector.get(AutoModeManager.class);

        if (Constants.kLoggingRobot) {
            robotLoopLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/Robot");
            looperLogger = new DoubleLogEntry(DataLogManager.getLog(), "Timings/RobotState");
        }
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
            DriverStation.silenceJoystickConnectionWarning(true);
            // Remember to register our subsystems below! The subsystem manager deals with calling
            // readFromHardware and writeToHardware on a loop, but it can only call read/write it if it
            // can recognize the subsystem. To recognize your subsystem, just add it alongside the
            // drive, ledManager, and camera parameters.
            subsystemManager.setSubsystems(drive, ledManager, camera);

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
                // start logging
                DataLogManager.start(logFileDir, "", Constants.kLooperDt);
                if (RobotBase.isReal()) {
                    Util.cleanLogFiles();
                }
                DriverStation.startDataLog(DataLogManager.getLog(), false);
            }

            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);

            // zeroing ypr - (-90) pigeon is mounted with the "y" axis facing forward
            infrastructure.resetPigeon(Rotation2d.fromDegrees(-90));
            subsystemManager.zeroSensors();

            /** [Specific subsystem] not zeroed on boot up - letting ppl know */
            faulted = true;

            /** Register inputHandler */
            inputHandler = Injector.get(InputHandler.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            /** Driver Commands */
            inputHandler.listenAction(
                    "DriverAction",
                    ActionState.PRESSED,
                    () -> {

                    }
            );
            /** Operator Commands */
            inputHandler.listenAction(
                    "OperatorAction",
                    ActionState.PRESSED,
                    () -> {

                    }
            );

            /** Buttonboard Commands */

            inputHandler.listenAction(
                    "ButtonBoardAction",
                    ActionState.PRESSED,
                    () -> {

                    }
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
            orchestrator.clearThreads();

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

        // TODO: Set up subsystem states

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

            SmartDashboard.putString("Git Hash", Constants.kGitHash);
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
                ledManager.indicateStatus(LedManager.RobotStatus.ZEROING);
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
                // TODO: Logic for if the robot is not a simulation
                // TODO: Also don't forget to add logic to make faulted false.
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
        inputHandler.update();

        drive.setTeleopInputs(
                    -inputHandler.getActionAsDouble("throttle"),
                    -inputHandler.getActionAsDouble("strafe"),
                     inputHandler.getActionAsDouble("rotation")
        );
    }

    /**
     * Actions to perform periodically when the robot is in the test period
     */
    @Override
    public void testPeriodic() {
    }
}
