package com.team1816.lib.subsystems.drive;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.lib.util.team254.SwerveDriveHelper;
import com.team1816.lib.util.team254.SwerveDriveSignal;
import com.team1816.season.configuration.Constants;
import com.team1816.season.hardware.components.ProxySensor;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.LedManager;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

/**
 * A class that models a Swerve drivetrain
 */
@Singleton
public class SwerveDrive extends Drive implements SwerveDrivetrain, PidProvider {

    /** Constants */

    /** Module Characterization */
    private static final double moduleDeltaX = kDriveWheelbaseLengthMeters / 2.0;
    private static final double moduleDeltaY = kDriveWheelTrackWidthMeters / 2.0;

    // module indices
    public static final int kFrontLeft = 0;
    public static final int kFrontRight = 1;
    public static final int kBackLeft = 2;
    public static final int kBackRight = 3;

    // module positions
    public static final Translation2d kFrontLeftModulePosition = new Translation2d(
        moduleDeltaX,
        moduleDeltaY
    );
    public static final Translation2d kFrontRightModulePosition = new Translation2d(
        moduleDeltaX,
        -moduleDeltaY
    );
    public static final Translation2d kBackLeftModulePosition = new Translation2d(
        -moduleDeltaX,
        moduleDeltaY
    );
    public static final Translation2d kBackRightModulePosition = new Translation2d(
        -moduleDeltaX,
        -moduleDeltaY
    );

    public static final Translation2d[] kModulePositions = {
        kFrontLeftModulePosition,
        kFrontRightModulePosition,
        kBackRightModulePosition,
        kBackLeftModulePosition,
    };

    // Kinematics (https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        kFrontLeftModulePosition,
        kFrontRightModulePosition,
        kBackLeftModulePosition,
        kBackRightModulePosition
    );

    /** Components */
    public SwerveModule[] swerveModules;

    /** Trajectory */
    protected List<Rotation2d> headingsList;
    protected int trajectoryIndex = 0;

    /** Odometry variables */
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDriveHelper swerveDriveHelper = new SwerveDriveHelper();

    /** States */
    public SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
    SwerveModuleState[] actualModuleStates = new SwerveModuleState[4];
    SwerveModulePosition[] actualModulePositions = new SwerveModulePosition[4];
    public double[] motorTemperatures = new double[4];

    /**
     * Instantiates a swerve drivetrain from base subsystem parameters
     * @param lm LEDManager
     * @param inf Infrastructure
     * @param rs RobotState
     * @see Drive#Drive(LedManager, Infrastructure, RobotState)
     */
    @Inject
    public SwerveDrive(LedManager lm, Infrastructure inf, RobotState rs) {
        super(lm, inf, rs);
        swerveModules = new SwerveModule[4];

        // enableDigital all Talons in open loop mode
        swerveModules[kFrontLeft] = factory.getSwerveModule(NAME, "frontLeft");
        swerveModules[kFrontRight] = factory.getSwerveModule(NAME, "frontRight");
        swerveModules[kBackLeft] = factory.getSwerveModule(NAME, "backLeft");
        swerveModules[kBackRight] = factory.getSwerveModule(NAME, "backRight");

        setOpenLoop(SwerveDriveSignal.NEUTRAL);

        actualModulePositions[kFrontLeft] = new SwerveModulePosition();
        actualModulePositions[kFrontRight] = new SwerveModulePosition();
        actualModulePositions[kBackLeft] = new SwerveModulePosition();
        actualModulePositions[kBackRight] = new SwerveModulePosition();

        swerveOdometry =
            new SwerveDriveOdometry(
                swerveKinematics,
                Constants.EmptyRotation2d,
                actualModulePositions
            );

    }

    /** Read/Write Periodic */

    /**
     * Writes outputs / demands to hardware on the drivetrain such as motors and handles the desired state of the modules.
     * Inputs sent to kinematics.
     * @see SwerveDriveKinematics
     */
    @Override
    public synchronized void writeToHardware() {
        if (controlState == ControlState.OPEN_LOOP) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredModuleStates,
                kOpenLoopMaxVelMeters
            );
            for (int i = 0; i < 4; i++) {
                swerveModules[i].setDesiredState(desiredModuleStates[i], true);
            }
        }
    }

    /**
     * Reads outputs from hardware on the drivetrain such as sensors and handles the actual state of the swerve modules and
     * drivetrain speeds. Used to update odometry and other related data.
     * @see Infrastructure
     * @see SwerveModule
     * @see RobotState
     */
    @Override
    public synchronized void readFromHardware() {
        for (int i = 0; i < 4; i++) {
            // logging actual angle and velocity of swerve motors (azimuth & drive)
            swerveModules[i].update();
            actualModuleStates[i] = swerveModules[i].getActualState();
            actualModulePositions[i] = swerveModules[i].getActualPosition();
            // logging current temperatures of each module's drive motor
            motorTemperatures[i] = swerveModules[i].getMotorTemp();
        }
        chassisSpeed = swerveKinematics.toChassisSpeeds(actualModuleStates);

        if (RobotBase.isSimulation()) {
            simulateGyroOffset();
        }
        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());

        swerveOdometry.update(actualHeading, actualModulePositions);


        updateRobotState();
    }

    /** General getters and setters */

    /**
     * Returns the list of headings for following a path that are transposed onto a path
     * @return trajectoryHeadings
     */
    public Rotation2d getTrajectoryHeadings() {
        if (headingsList == null) {
            return Constants.EmptyRotation2d;
        } else if (trajectoryIndex > headingsList.size() - 1) {
            //System.out.println("heck the headings aren't long enough");
            return Constants.EmptyRotation2d;
        }
        if (
            getTrajectoryTimestamp() >
            trajectory.getStates().get(trajectoryIndex).timeSeconds ||
            trajectoryIndex == 0
        ) trajectoryIndex++;
        if (trajectoryIndex >= headingsList.size()) {
            System.out.println(headingsList.get(headingsList.size() - 1) + " = max");
            return headingsList.get(headingsList.size() - 1);
        }
        double timeBetweenPoints =
            (
                trajectory.getStates().get(trajectoryIndex).timeSeconds -
                trajectory.getStates().get(trajectoryIndex - 1).timeSeconds
            );
        Rotation2d heading;
        heading =
            headingsList
                .get(trajectoryIndex - 1)
                .interpolate(
                    headingsList.get(trajectoryIndex),
                    getTrajectoryTimestamp() / timeBetweenPoints
                );
        return heading;
    }

    /**
     * Starts a trajectory to be followed with headings (rotate while moving)
     * @param trajectory Trajectory
     * @param headings Headings (for swerve)
     * @see Drive#startTrajectory(Trajectory, List)
     */
    @Override
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        super.startTrajectory(trajectory, headings);
        headingsList = headings;
        trajectoryIndex = 0;
    }

    /**
     * Sets the module states to a desired set of states in closed loop - this is used during autos
     * @param desiredStates desiredModuleStates
     * @see com.team1816.lib.auto.actions.TrajectoryAction
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (controlState != ControlState.TRAJECTORY_FOLLOWING) {
            controlState = ControlState.TRAJECTORY_FOLLOWING;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates,
            (kPathFollowingMaxVelMeters)
        );
        desiredModuleStates = desiredStates;
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(desiredStates[i], false);
        }
    }

    /**
     * Updates robotState based on values from odometry and sensor readings in readFromHardware
     * @see RobotState
     */
    @Override
    public void updateRobotState() {
        robotState.fieldToVehicle = swerveOdometry.getPoseMeters();

        var cs = new ChassisSpeeds(
            chassisSpeed.vxMetersPerSecond,
            chassisSpeed.vyMetersPerSecond,
            chassisSpeed.omegaRadiansPerSecond
        );
        robotState.calculatedVehicleAccel =
            new ChassisSpeeds(
                (cs.vxMetersPerSecond - robotState.deltaVehicle.vxMetersPerSecond) /
                Constants.kLooperDt,
                (cs.vyMetersPerSecond - robotState.deltaVehicle.vyMetersPerSecond) /
                Constants.kLooperDt,
                -9.80
            );
        robotState.deltaVehicle = cs;
        // check if motors are overheating - update robotState
        SmartDashboard.putNumber("Drive/Temperature", motorTemperatures[0]);
        robotState.drivetrainTemp = motorTemperatures[0];
    }

    /** Open Loop control */

    /**
     * Sets the ControlState to OPEN_LOOP and modifies the desired SwerveModuleState based on the DriveSignal
     * @param signal DriveSignal
     * @see SwerveDriveSignal
     */
    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (controlState != ControlState.OPEN_LOOP) {
            System.out.println("switching to open loop");
            controlState = ControlState.OPEN_LOOP;
        }
        SwerveModuleState[] desiredStatesSignal = new SwerveModuleState[4];
        for (int i = 0; i < desiredStatesSignal.length; i++) {
            desiredStatesSignal[i] =
                new SwerveModuleState(
                    ((SwerveDriveSignal) signal).getWheelSpeeds()[i],
                    ((SwerveDriveSignal) signal).getWheelAzimuths()[i]
                );
        }

        desiredModuleStates = desiredStatesSignal;
    }

    /**
     * Translates tele-op inputs into a SwerveDriveSignal to be used in setOpenLoop()
     * @param forward forward demand
     * @param strafe strafe demand
     * @param rotation rotation demand
     * @see this#setOpenLoop(DriveSignal)
     * @see Drive#setTeleopInputs(double, double, double)
     * @see SwerveDriveSignal
     */
    @Override
    public void setTeleopInputs(double forward, double strafe, double rotation) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        SwerveDriveSignal signal = swerveDriveHelper.calculateDriveSignal(
            (isDemoMode ? forward * demoModeMultiplier : forward),
            (isDemoMode ? strafe * demoModeMultiplier : strafe),
            (isDemoMode ? rotation * demoModeMultiplier : rotation),
            isSlowMode,
            true,
            false
        );

        // To avoid overriding brake command
        if (!isBraking) {
            setOpenLoop(signal);
        }
    }

    /**
     * Sets whether the drivetrain is braking
     * @param braking boolean
     * @see Drive#setBraking(boolean)
     */
    @Override
    public synchronized void setBraking(boolean braking) {
        isBraking = braking;
        if (braking) {
            setOpenLoop(SwerveDriveSignal.BRAKE);
        }
    }

    /** general getters */

    /**
     * Returns the SwerveModules associated with the drivetrain
     * @return SwerveModule[]
     */
    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    /**
     * Returns the actual module states of the drivetrain with the help of kinematics
     * @return SwerveModuleState[]
     * @see SwerveDriveKinematics#toSwerveModuleStates(ChassisSpeeds, Translation2d)
     */
    public SwerveModuleState[] getStates() {
        return swerveKinematics.toSwerveModuleStates(chassisSpeed);
    }

    /**
     * Resets the odometry calculations to a specific pose (typically used in parallel with a vision processing env)
     * to accurately re-localize position
     * @param pose Pose2d
     * @see Drive#resetOdometry(Pose2d)
     */
    @Override
    public void resetOdometry(Pose2d pose) {
        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());
        swerveOdometry.resetPosition(actualHeading, actualModulePositions, pose);
        swerveOdometry.update(actualHeading, actualModulePositions);
        updateRobotState();
    }

    /**
     * Zeroes the azimuth sensors of the swerve modules based on a certain pose
     * @param pose Pose2d
     * @see SwerveModule#zeroAzimuthSensor()
     * @see Drive#zeroSensors()
     */
    @Override
    public void zeroSensors(Pose2d pose) {
        System.out.println("Zeroing drive sensors!");

        // resetting ACTUAL module states
        for (int i = 0; i < 4; i++) {
            actualModuleStates[i] = new SwerveModuleState();
            swerveModules[i].zeroAzimuthSensor();
        }

        resetOdometry(pose);
        startingPose = pose;
        chassisSpeed = new ChassisSpeeds();
        isBraking = false;
    }

    /**
     * Stops the drivetrain
     * @see Drive#stop()
     */
    @Override
    public synchronized void stop() {
        for (int i = 0; i < 4; i++) {
            SwerveModuleState stoppedState = new SwerveModuleState(
                0,
                swerveModules[i].getActualState().angle
            );
            swerveModules[i].setDesiredState(
                    stoppedState,
                    controlState == ControlState.OPEN_LOOP
                );
            desiredModuleStates[i] = stoppedState;
        }
    }

    /** config and tests */

    /**
     * Tests the drivetrain by testing each swerve module
     * @return true if tests passed
     * @see SwerveModule#checkSystem()
     * @see Drive#testSubsystem()
     */
    @Override
    public boolean testSubsystem() {
        boolean modulesPassed = true;
        for (SwerveModule swerveModule : swerveModules) {
            if (!swerveModule.checkSystem()) {
                modulesPassed = false;
                break;
            }
        }

        return modulesPassed;
    }

    /**
     * Returns the drive motor pid configuration of the drivetrain
     * @return PIDSlotConfiguration
     * @see Drive#getPIDConfig()
     */
    @Override
    public PIDSlotConfiguration getPIDConfig() {
        PIDSlotConfiguration defaultPIDConfig = new PIDSlotConfiguration();
        defaultPIDConfig.kP = 0.0;
        defaultPIDConfig.kI = 0.0;
        defaultPIDConfig.kD = 0.0;
        defaultPIDConfig.kF = 0.0;
        return (factory.getSubsystem(NAME).implemented)
            ? factory
                .getSubsystem(NAME)
                .swerveModules.drivePID.getOrDefault("slot0", defaultPIDConfig)
            : defaultPIDConfig;
    }

    /**
     * Returns the associated kinematics of the drivetrain
     * @return swerveKinematics
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveKinematics;
    }
}
