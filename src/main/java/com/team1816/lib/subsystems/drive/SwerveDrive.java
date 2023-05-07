package com.team1816.lib.subsystems.drive;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.Symmetry;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.lib.util.team254.SwerveDriveHelper;
import com.team1816.lib.util.team254.SwerveDriveSignal;
import com.team1816.season.Robot;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.List;
import java.util.Objects;

/**
 * A class that models a Swerve drivetrain
 */
@Singleton
public class SwerveDrive extends Drive implements SwerveDrivetrain, PidProvider {

    /** Constants */

    /**
     * Module Characterization
     */
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

    /**
     * Components
     */
    public SwerveModule[] swerveModules;

    /**
     * Trajectory
     */
    protected List<Rotation2d> headingsList;
    protected int trajectoryIndex = 0;

    /**
     * Odometry variables
     */
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDriveHelper swerveDriveHelper = new SwerveDriveHelper();

    /**
     * States
     */
    public SwerveModuleState[] desiredModuleStates = new SwerveModuleState[4];
    SwerveModuleState[] actualModuleStates = new SwerveModuleState[4];
    SwerveModulePosition[] actualModulePositions = new SwerveModulePosition[4];
    public double[] motorTemperatures = new double[4];

    /**
     * Logging
     */
    private DoubleLogEntry temperatureLogger;

    /**
     * Instantiates a swerve drivetrain from base subsystem parameters
     *
     * @param lm  LEDManager
     * @param inf Infrastructure
     * @param rs  RobotState
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

        if (Constants.kLoggingDrivetrain) {
            desStatesLogger = new DoubleArrayLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/DesStates");
            actStatesLogger = new DoubleArrayLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/ActStates");
            temperatureLogger = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/Temperature");
            gyroPitchLogger = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/Pitch");
            gyroRollLogger = new DoubleLogEntry(DataLogManager.getLog(), "Drivetrain/Swerve/Roll");
        }
    }

    /** Read/Write Periodic */

    /**
     * Writes outputs / demands to hardware on the drivetrain such as motors and handles the desired state of the modules.
     * Inputs sent to kinematics.
     *
     * @see SwerveDriveKinematics
     */
    @Override
    public synchronized void writeToHardware() {
        if (controlState == ControlState.OPEN_LOOP) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredModuleStates,
                kMaxVelOpenLoopMeters
            );
            for (int i = 0; i < 4; i++) {
                swerveModules[i].setDesiredState(desiredModuleStates[i], true);
            }
        }
    }

    /**
     * Reads outputs from hardware on the drivetrain such as sensors and handles the actual state of the swerve modules and
     * drivetrain speeds. Used to update odometry and other related data.
     *
     * @see Infrastructure
     * @see SwerveModule
     * @see RobotState
     */
    @Override
    public synchronized void readFromHardware() {
        double[] actualStates = new double[8];
        double[] desiredStates = new double[8];
        for (int i = 0; i < 4; i++) {
            // logging actual angle and velocity of swerve motors (azimuth & drive)
            swerveModules[i].update();
            actualModuleStates[i] = swerveModules[i].getActualState();
            actualModulePositions[i] = swerveModules[i].getActualPosition();
            // logging current temperatures of each module's drive motor
            motorTemperatures[i] = swerveModules[i].getMotorTemp();

            if (Constants.kLoggingDrivetrain) {
                // populating double list with actState angles and speeds
                actualStates[i * 2] = actualModuleStates[i].angle.getRadians();
                actualStates[i * 2 + 1] = actualModuleStates[i].speedMetersPerSecond;

                // populating double list with desState angles and speeds
                desiredStates[i * 2] = desiredModuleStates[i].angle.getRadians();
                desiredStates[i * 2 + 1] = desiredModuleStates[i].speedMetersPerSecond;
            }
        }
        chassisSpeed = swerveKinematics.toChassisSpeeds(actualModuleStates);

        if (RobotBase.isSimulation()) {
            simulateGyroOffset();
        }
        actualHeading = Rotation2d.fromDegrees(infrastructure.getYaw());

        swerveOdometry.update(actualHeading, actualModulePositions);

        if (Constants.kLoggingDrivetrain) {
            ((DoubleArrayLogEntry) desStatesLogger).append(desiredStates);
            ((DoubleArrayLogEntry) actStatesLogger).append(actualStates);
        }

        updateRobotState();
    }

    /** General getters and setters */

    /**
     * Returns the list of headings for following a path that are transposed onto a path
     *
     * @return trajectoryHeadings
     */
    public Rotation2d getTrajectoryHeadings() {
        if (headingsList == null) {
            return Constants.EmptyRotation2d;
        } else if (trajectoryIndex > headingsList.size() - 1) {
            return headingsList.get(headingsList.size() - 1);
        }
        if (
            getTrajectoryTimestamp() >
                trajectory.getStates().get(trajectoryIndex).timeSeconds ||
                trajectoryIndex == 0
        ) trajectoryIndex++;
        if (trajectoryIndex >= headingsList.size()) {
            GreenLogger.log(headingsList.get(headingsList.size() - 1) + " = max");
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
     *
     * @param trajectory Trajectory
     * @param headings   Headings (for swerve)
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
     *
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
     * Sub-container of gyroscopic based balancing with manual adjustment factors for a swerve drivetrain
     *
     * @see Drive#autoBalance(ChassisSpeeds)
     */
    @Override
    public void autoBalance(ChassisSpeeds fieldRelativeChassisSpeeds) {
        double pitch = -infrastructure.getPitch();
        double roll = infrastructure.getRoll();
        double throttle = 0;
        double strafe = 0;
        var heading = Constants.EmptyRotation2d;

        double threshold = Constants.autoBalanceThresholdDegrees;

        double autoBalanceDivider = Constants.autoBalanceDivider;

        if (Math.abs(pitch) > threshold || Math.abs(roll) > threshold) {
            throttle = pitch / autoBalanceDivider;
            strafe = roll / autoBalanceDivider;
        }

        // if not braking and ((throttle || strafe != 0) or joystick strafe input != 0), auto-balance
        // Else, lock wheels to face left/right side of field
        if (isBraking || ((throttle == 0 && strafe == 0) && Objects.equals(fieldRelativeChassisSpeeds, new ChassisSpeeds()))) {
            heading = Rotation2d.fromDegrees(90).minus(robotState.fieldToVehicle.getRotation());
            SwerveModuleState templateState = new SwerveModuleState(0, heading);
            SwerveModuleState[] statePassIn = new SwerveModuleState[]{templateState, templateState, templateState, templateState};
            setModuleStates(statePassIn);
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                throttle + fieldRelativeChassisSpeeds.vxMetersPerSecond,
                strafe + fieldRelativeChassisSpeeds.vyMetersPerSecond,
                fieldRelativeChassisSpeeds.omegaRadiansPerSecond);
            setModuleStates(swerveKinematics.toSwerveModuleStates(chassisSpeeds));
        }
    }

    /**
     * Updates robotState based on values from odometry and sensor readings in readFromHardware
     *
     * @see RobotState
     */
    @Override
    public void updateRobotState() {
        robotState.fieldToVehicle = swerveOdometry.getPoseMeters();
        robotState.driverRelativeFieldToVehicle = new Pose2d( // for inputs ONLY
            robotState.fieldToVehicle.getTranslation(),
            (robotState.allianceColor == Color.BLUE && Constants.fieldSymmetry == Symmetry.AXIS) ? robotState.fieldToVehicle.getRotation() : robotState.fieldToVehicle.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );

        var cs = new ChassisSpeeds(
            chassisSpeed.vxMetersPerSecond,
            chassisSpeed.vyMetersPerSecond,
            chassisSpeed.omegaRadiansPerSecond
        );
        robotState.calculatedVehicleAccel =
            new ChassisSpeeds(
                (cs.vxMetersPerSecond - robotState.deltaVehicle.vxMetersPerSecond) /
                    Robot.looperDt,
                (cs.vyMetersPerSecond - robotState.deltaVehicle.vyMetersPerSecond) /
                    Robot.looperDt,
                -9.80
            );
        robotState.deltaVehicle = cs;

        temperatureLogger.append(motorTemperatures[0]);
        robotState.drivetrainTemp = motorTemperatures[0];

        robotState.vehicleToFloorProximityCentimeters = infrastructure.getMaximumProximity();

        if (Constants.kLoggingDrivetrain) {
            drivetrainPoseLogger.append(new double[]{robotState.fieldToVehicle.getX(), robotState.fieldToVehicle.getY(), robotState.fieldToVehicle.getRotation().getDegrees()});
            drivetrainChassisSpeedsLogger.append(new double[]{robotState.deltaVehicle.vxMetersPerSecond, robotState.deltaVehicle.vyMetersPerSecond, robotState.deltaVehicle.omegaRadiansPerSecond});
            gyroPitchLogger.append(infrastructure.getPitch());
            gyroRollLogger.append(infrastructure.getRoll());
        }
    }

    /** Open Loop control */

    /**
     * Sets the ControlState to OPEN_LOOP and modifies the desired SwerveModuleState based on the DriveSignal
     *
     * @param signal DriveSignal
     * @see SwerveDriveSignal
     */
    @Override
    public void setOpenLoop(DriveSignal signal) {
        if (controlState != ControlState.OPEN_LOOP) {
            GreenLogger.log("switching to open loop");
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
     *
     * @param forward  forward demand
     * @param strafe   strafe demand
     * @param rotation rotation demand
     * @see this#setOpenLoop(DriveSignal)
     * @see Drive#setTeleopInputs(double, double, double)
     * @see SwerveDriveSignal
     */
    @Override
    public void setTeleopInputs(double forward, double strafe, double rotation) {
        SwerveDriveSignal signal;
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }

        if (forward == 0 && strafe == 0 && rotation == 0) {

            Rotation2d[] azimuths = new Rotation2d[4];

            for (int i = 0; i < 4; i++) {
                azimuths[i] = Rotation2d.fromDegrees(swerveModules[i].azimuthActualDeg);
            }

            signal = new SwerveDriveSignal(new double[]{0, 0, 0, 0}, azimuths, false);
        } else {
            signal = swerveDriveHelper.calculateDriveSignal(
                (isDemoMode ? forward * demoModeMultiplier : forward),
                (isDemoMode ? strafe * demoModeMultiplier : strafe),
                (isDemoMode ? rotation * demoModeMultiplier : rotation),
                isSlowMode,
                true,
                false
            );
        }

        // To avoid overriding brake command
        if (!isBraking) {
            setOpenLoop(signal);
        }
    }

    /**
     * Sets whether the drivetrain is braking
     *
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
     *
     * @return SwerveModule[]
     */
    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    /**
     * Returns the actual module states of the drivetrain with the help of kinematics
     *
     * @return SwerveModuleState[]
     * @see SwerveDriveKinematics#toSwerveModuleStates(ChassisSpeeds, Translation2d)
     */
    public SwerveModuleState[] getStates() {
        return swerveKinematics.toSwerveModuleStates(chassisSpeed);
    }

    /**
     * Resets the odometry calculations to a specific pose (typically used in parallel with a vision processing env)
     * to accurately re-localize position
     *
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
     *
     * @param pose Pose2d
     * @see SwerveModule#zeroAzimuthSensor()
     * @see Drive#zeroSensors()
     */
    @Override
    public void zeroSensors(Pose2d pose) {
        GreenLogger.log("Zeroing drive sensors!");

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
     *
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
     *
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
     *
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
     *
     * @return swerveKinematics
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveKinematics;
    }
}
