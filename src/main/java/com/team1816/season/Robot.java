package com.team1816.season;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.input_handler.*;
import com.team1816.lib.loops.Looper;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.SubsystemLooper;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.auto.AutoModeManager;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.DrivetrainTargets;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;

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

    public Elevator.ANGLE_STATE prevAngleState;

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
                DriverStation.startDataLog(DataLogManager.getLog(), true);
            }

            subsystemManager.registerEnabledLoops(enabledLoop);
            subsystemManager.registerDisabledLoops(disabledLoop);

            // zeroing ypr - (-90) b/c our pigeon is mounted with the "y" axis facing forward
            infrastructure.resetPigeon(Rotation2d.fromDegrees(-90));
            subsystemManager.zeroSensors();
            faulted = true; // elevator not zeroed on bootup - letting ppl know

            /** Register inputHandler */
            inputHandler = Injector.get(InputHandler.class);
            DriverStation.silenceJoystickConnectionWarning(true);

            /** Driver Commands */

            //zeroPose
            inputHandler.listenDriverButton(
                    Button.START,
                    Button.State.PRESSED,
                    () ->
                        drive.zeroSensors(
                                robotState.allianceColor == Color.BLUE ?
                                Constants.kDefaultZeroingPose :
                                Constants.kFlippedZeroingPose
                        )
            );

            //autoTargetAlign
            inputHandler.listenDriverButton(
                    Button.A,
                    Button.State.PRESSED,
                    () -> {
                        if (robotState.allianceColor == Color.BLUE) {
                            robotState.target = DrivetrainTargets.blueTargets.get(grid * 3 + node);
                        } else {
                            robotState.target = DrivetrainTargets.redTargets.get(grid * 3 + node);
                        }
                        orchestrator.autoTargetAlign(level);
                    }
            );

            //brakeMode
            inputHandler.listenOperatorButtonPressAndRelease(
                    Button.B,
                    drive::setBraking
            );

            //slowMode
            inputHandler.listenDriverTriggerPressAndRelease(
                    Trigger.RIGHT,
                    drive::setSlowMode
            );

            //autoBalance
            inputHandler.listenDriverTriggerPressAndRelease(
                    Trigger.LEFT,
                    (pressed) -> {
                        drive.setAutoBalance(pressed);

                        if (pressed) {
                            ledManager.indicateStatus(LedManager.RobotStatus.BALANCE, LedManager.ControlState.BLINK);
                        } else {
                            ledManager.indicateStatus(LedManager.RobotStatus.BALANCE, LedManager.ControlState.SOLID);
                        }
                    }
            );

            // intakeCone
            inputHandler.listenDriverButton(
                    Button.RIGHT_BUMPER,
                    Button.State.PRESSED,
                    () -> {
                        if (
                            elevator.getDesiredAngleState() == Elevator.ANGLE_STATE.SHELF_COLLECT
                        ) { // collects from shelf
                            collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.SHELF);
                        } else { // collects from floor
                            collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CONE, Collector.PIVOT_STATE.FLOOR);
                        }

                        ledManager.indicateStatus(LedManager.RobotStatus.CONE, LedManager.ControlState.BLINK); // indicates on LEDs
                    }
            );

            //intakeCone stop
            inputHandler.listenDriverButton(
                    Button.RIGHT_BUMPER,
                    Button.State.RELEASED,
                    () -> {
                        collector.setDesiredState(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW);
                        ledManager.indicateStatus(LedManager.RobotStatus.CONE, LedManager.ControlState.SOLID);
                    }
            );

            //intakeCube
            inputHandler.listenDriverButton(
                    Button.LEFT_BUMPER,
                    Button.State.PRESSED,
                    () -> {
                        if (
                            elevator.getDesiredAngleState() == Elevator.ANGLE_STATE.SHELF_COLLECT
                        ) { // collects from shelf
                            collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CUBE, Collector.PIVOT_STATE.SHELF);
                        } else { // collects from floor
                            collector.setDesiredState(Collector.ROLLER_STATE.INTAKE_CUBE, Collector.PIVOT_STATE.FLOOR);
                        }
                        ledManager.indicateStatus(LedManager.RobotStatus.CUBE, LedManager.ControlState.BLINK); // indicates on LEDs
                    }
            );

            //intakeCube stop
            inputHandler.listenDriverButton(
                    Button.LEFT_BUMPER,
                    Button.State.RELEASED,
                    () -> {
                        collector.setDesiredState(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW);
                        ledManager.indicateStatus(LedManager.RobotStatus.CUBE, LedManager.ControlState.SOLID);
                    }
            );

            //toggleArmScoreCollect
            inputHandler.listenDriverButton(
                    Button.X,
                    Button.State.HELD,
                    () -> {
                        if (elevator.getDesiredAngleState() == Elevator.ANGLE_STATE.SHELF_COLLECT
                            && robotState.actualElevatorExtensionState != Elevator.EXTENSION_STATE.MIN) {
                            elevator.setDesiredState(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN);
                        } else if (elevator.getDesiredAngleState() != Elevator.ANGLE_STATE.STOW) {
                            elevator.setDesiredState(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN);
                        } else {
                            elevator.setDesiredState(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN);
                            collector.setDesiredPivotState(Collector.PIVOT_STATE.STOW);
                        }
                    }
            );

            //shelfPos
            inputHandler.listenDriverButton(
                    Button.Y,
                    Button.State.PRESSED,
                    () ->
                        elevator.setDesiredState(
                                Elevator.ANGLE_STATE.SHELF_COLLECT,
                                Elevator.EXTENSION_STATE.SHELF_COLLECT
                        )
            );

            //snapToHumanPlayer
            inputHandler.listenDriverDpadPressAndRelease(
                    Dpad.UP,
                    (pressed) -> {
                        snappingToHumanPlayer = pressed;
                        snappingToDriver = false;
                    }
            );

            // snapToDriver
            inputHandler.listenDriverDpadPressAndRelease(
                    Dpad.DOWN,
                    (pressed) -> {
                        snappingToDriver = pressed;
                        snappingToHumanPlayer = false;
                    }
            );

            /** Operator Commands */

            //updatePoseWithCamera
            inputHandler.listenOperatorButton(
                    Button.LEFT_BUMPER,
                    Button.State.PRESSED,
                    orchestrator::updatePoseWithCamera
            );

            //outtake
            inputHandler.listenOperatorButtonPressAndRelease(
                    Button.RIGHT_BUMPER,
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
            );

            //extendMin
            inputHandler.listenOperatorButton(
                    Button.A,
                    Button.State.PRESSED,
                    () -> {
                        GreenLogger.log("extend min");
                        orchestrator.alignMin();
                    }
            );

            //extendMid
            inputHandler.listenOperatorButton(
                    Button.X,
                    Button.State.PRESSED,
                    () -> {
                        GreenLogger.log("extend mid");
                        orchestrator.alignMid();
                    }
            );

            //extendMax
            inputHandler.listenOperatorButton(
                    Button.Y,
                    Button.State.PRESSED,
                    () -> {
                        GreenLogger.log("extend max");
                        orchestrator.alignMax();
                    }
            );

            //autoScore
            inputHandler.listenOperatorButton(
                    Button.B,
                    Button.State.PRESSED,
                    () -> {
                        GreenLogger.log("auto score");
                        orchestrator.autoScore();
                    }
            );

            //toggleCollectorPivot
            inputHandler.listenOperatorButton(
                    Button.START,
                    Button.State.PRESSED,
                    () -> {
                        if (robotState.actualElevatorAngleState == Elevator.ANGLE_STATE.SCORE) {
                            if (collector.getDesiredPivotState() == Collector.PIVOT_STATE.SCORE) {
                                collector.setDesiredState(collector.getDesiredRollerState(), Collector.PIVOT_STATE.STOW);
                            } else if (collector.getDesiredPivotState() == Collector.PIVOT_STATE.STOW) {
                                collector.setCurrentGameElement(Collector.GAME_ELEMENT.CONE);
                                collector.setDesiredState(collector.getDesiredRollerState(), Collector.PIVOT_STATE.SCORE);
                            }
                        }
                    }
            );

            /** Button Board Commands */

            //grid1
            inputHandler.listenButtonBoardButton(
                    Button.UP_LEFT,
                    Button.State.PRESSED,
                    () -> {
                        grid = robotState.allianceColor == Color.RED ? 0 : 2;
                        GreenLogger.log("Grid changed to FEEDER");
                    }
            );

            //grid2
            inputHandler.listenButtonBoardButton(
                    Button.UP,
                    Button.State.PRESSED,
                    () -> {
                        grid = 1;
                        GreenLogger.log("Grid changed to BALANCE");
                    }
            );

            //grid3
            inputHandler.listenButtonBoardButton(
                    Button.UP_RIGHT,
                    Button.State.PRESSED,
                    () -> {
                        grid = robotState.allianceColor == Color.RED ? 2 : 0;
                        GreenLogger.log("Grid changed to WALL");
                    }
            );

            //node1
            inputHandler.listenButtonBoardButton(
                    Button.LEFT,
                    Button.State.PRESSED,
                    () -> {
                        node = robotState.allianceColor == Color.RED ? 0 : 2;
                        GreenLogger.log("Node changed to LEFT");
                    }
            );

            //node2
            inputHandler.listenButtonBoardButton(
                    Button.CENTER,
                    Button.State.PRESSED,
                    () -> {
                        node = 1;
                        GreenLogger.log("Node changed to CENTER");
                    }
            );

            //node3
            inputHandler.listenButtonBoardButton(
                    Button.RIGHT,
                    Button.State.PRESSED,
                    () -> {
                        node = robotState.allianceColor == Color.RED ? 2 : 0;
                        GreenLogger.log("Node changed to RIGHT");
                    }
            );

            //level1
            inputHandler.listenButtonBoardButton(
                    Button.DOWN_LEFT,
                    Button.State.PRESSED,
                    () -> {
                        level = Elevator.EXTENSION_STATE.MIN;
                        GreenLogger.log("Score level changed to Low");
                    }
            );

            //level2
            inputHandler.listenButtonBoardButton(
                    Button.DOWN,
                    Button.State.PRESSED,
                    () -> {
                        level = Elevator.EXTENSION_STATE.MID;
                        GreenLogger.log("Score level changed to Mid");
                    }
            );

            //level3
            inputHandler.listenButtonBoardButton(
                    Button.DOWN_RIGHT,
                    Button.State.PRESSED,
                    () -> {
                        level = Elevator.EXTENSION_STATE.MAX;
                        GreenLogger.log("Score level changed to Max");
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

        collector.currentGameElement = Collector.GAME_ELEMENT.CONE;
        robotState.actualGameElement = Collector.GAME_ELEMENT.CONE;
        elevator.setDesiredState(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN);
        collector.setDesiredState(Collector.ROLLER_STATE.STOP, Collector.PIVOT_STATE.STOW);

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
        inputHandler.update();

        double strafe = inputHandler.getDriverAxisAsDouble(Axis.LEFT_HORIZONTAL);
        double throttle = inputHandler.getDriverAxisAsDouble(Axis.LEFT_VERTICAL);

        if (drive.isAutoBalancing()) {
            ChassisSpeeds fieldRelativeChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                    0,
                    -strafe,
                    0,
                    robotState.driverRelativeFieldToVehicle.getRotation());
            drive.autoBalance(fieldRelativeChassisSpeed);
        } else {
            double rotation = inputHandler.getDriverAxisAsDouble(Axis.RIGHT_HORIZONTAL);
            if (snappingToDriver || snappingToHumanPlayer) {
                double rotVal = MathUtil.inputModulus(
                        robotState.driverRelativeFieldToVehicle.getRotation().getDegrees(), robotState.allianceColor == Color.BLUE ? -180 : 180, robotState.allianceColor == Color.BLUE ? 180 : -180
                );
                if (snappingToDriver) {
                    if (rotVal == 0)
                        rotVal += 0.01d;
                    rotation = Math.min(0.5, (180 - Math.abs(rotVal)) / 40) * -Math.signum(rotVal);
                } else {
                    rotation = Math.min(0.5, Math.abs(rotVal) / 40) * Math.signum(rotVal);
                }
            }

            drive.setTeleopInputs(
                    -throttle,
                    -strafe,
                    rotation
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
