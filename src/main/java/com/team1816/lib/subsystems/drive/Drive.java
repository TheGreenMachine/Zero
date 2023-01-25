package com.team1816.lib.subsystems.drive;

import com.google.inject.Inject;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.loops.ILooper;
import com.team1816.lib.loops.Loop;
import com.team1816.lib.subsystems.PidProvider;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import com.team1816.season.subsystems.LedManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

/**
 * Base Drive class that TankDrive and SwerveDrive extend from
 *
 * @see TankDrive
 * @see SwerveDrive
 */
public abstract class Drive
    extends Subsystem
    implements TrackableDrivetrain, PidProvider {

    /**
     * Base factory, extended by {@link com.team1816.lib.DriveFactory} to determine the type of drivetrain with the help of
     * the Injector
     *
     * @see com.team1816.lib.Injector
     */
    public interface Factory {
        Drive getInstance();
    }

    /**
     * Properties
     */
    public static final String NAME = "drivetrain";

    /**
     * Demo Mode
     */
    protected double demoModeMultiplier;
    protected SendableChooser<DemoMode> demoModeChooser;
    protected DemoMode desiredMode;
    protected static final boolean isDemoMode =
        factory.getConstant(NAME, "isDemoMode", 0) > 0;

    /**
     * Components
     */
    protected static LedManager ledManager;

    /**
     * Localized state
     */
    protected ControlState controlState = ControlState.OPEN_LOOP;
    protected Rotation2d actualHeading = Constants.EmptyRotation2d;
    protected Rotation2d desiredHeading = new Rotation2d(); // only updated in trajectory following
    protected Pose2d desiredPose = new Pose2d(); // only updated in trajectory following
    protected ChassisSpeeds chassisSpeed = new ChassisSpeeds();

    protected boolean isBraking;
    protected boolean isSlowMode;

    /**
     * Trajectory
     */
    protected double trajectoryStartTime = 0;

    protected Pose2d startingPose = Constants.kDefaultZeroingPose;
    protected Trajectory trajectory;
    protected static double timestamp;

    /**
     * Simulator
     */
    protected double gyroDrift;
    protected final double tickRatioPerLoop = Constants.kLooperDt / .01d;

    /**
     * Constants
     */
    public static final double maxVelTicks100ms = factory.getConstant(
        NAME,
        "maxVelTicks100ms"
    );
    public static final double driveEncPPR = factory.getConstant(NAME, "encPPR");

    // Drivetrain characterization
    public static final double kDriveWheelTrackWidthInches = factory.getConstant(
        NAME,
        "trackWidth",
        22
    );
    public static final double kDriveWheelbaseLengthInches = factory.getConstant(
        NAME,
        "wheelbaseLength",
        22
    );
    public static final double kDriveWheelDiameterInches = factory.getConstant(
        NAME,
        "wheelDiameter"
    );
    public static final double kWheelCircumferenceInches =
        kDriveWheelDiameterInches * Math.PI;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;

    public static final double kDriveWheelTrackWidthMeters = Units.inchesToMeters(
        kDriveWheelTrackWidthInches
    );
    public static final double kDriveWheelbaseLengthMeters = Units.inchesToMeters(
        kDriveWheelbaseLengthInches
    );
    public static final double kDriveWheelDiameterMeters = Units.inchesToMeters(
        kDriveWheelDiameterInches
    );
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(
        kWheelCircumferenceInches
    );
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(
        kDriveWheelRadiusInches
    );
    public static double kTrackScrubFactor = factory.getConstant(
        NAME,
        "kTrackScrubFactor"
    );
    // Drive speed
    public static final double kPathFollowingMaxAccelMeters = factory.getConstant(
        NAME,
        "maxAccel",
        4
    );
    public static final double kPathFollowingMaxVelMeters = factory.getConstant(
        NAME,
        "maxVelPathFollowing"
    );
    public static final double kOpenLoopMaxVelMeters = factory.getConstant(
        NAME,
        "maxVelOpenLoop"
    );

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 4;
    public static final double kMaxAngularSpeed = factory.getConstant(NAME, "maxRotVel"); // rad/sec
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared =
        2 * Math.PI;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularAccelerationRadiansPerSecondSquared
    );

    /**
     * Instantiates the Drive with base subsystem parameters and accounts for DemoMode
     *
     * @param lm  LEDManager
     * @param inf Infrastructure
     * @param rs  RobotState
     */
    @Inject
    public Drive(LedManager lm, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        ledManager = lm;

        if (isDemoMode) {
            demoModeChooser = new SendableChooser<>();
            SmartDashboard.putData("Demo Mode", demoModeChooser);
            // demo mode functionality configuration

            System.out.println("    Using Demo Control Board");

            demoModeChooser = new SendableChooser<>();
            SmartDashboard.putData("Demo Mode", demoModeChooser);

            for (DemoMode demoMode : DemoMode.values()) {
                demoModeChooser.addOption(demoMode.name(), demoMode);
            }
            demoModeChooser.setDefaultOption(DemoMode.SLOW.name(), DemoMode.SLOW);
            demoModeMultiplier = 0.25;
        }
    }

    /**
     * Registers enabled loops for the drivetrain and sorts between the control modes of OPEN_LOOP and TRAJECTORY_FOLLOWING
     *
     * @param in looper
     * @see com.team1816.lib.loops.Looper
     * @see com.team1816.lib.subsystems.SubsystemLooper
     */
    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(
            new Loop() {
                @Override
                public void onStart(double timestamp) {
                }

                @Override
                public void onLoop(double timestamp) {
                    synchronized (Drive.this) {
                        Drive.timestamp = timestamp;
                        switch (controlState) {
                            case OPEN_LOOP:
                                updateOpenLoopPeriodic();
                                break;
                            case TRAJECTORY_FOLLOWING:
                                updateTrajectoryPeriodic(timestamp);
                                break;
                            default:
                                System.out.println(
                                    "unexpected drive control state: " + controlState
                                );
                                break;
                        }
                    }
                }

                @Override
                public void onStop(double timestamp) {
                    stop();
                }
            }
        );
    }

    /** base methods */

    /**
     * Starts the drivetrain to follow a trajectory and sets drivetrain state
     *
     * @param trajectory Trajectory
     * @param headings   Headings (for swerve)
     * @see Trajectory
     */
    public void startTrajectory(Trajectory trajectory, List<Rotation2d> headings) {
        controlState = ControlState.TRAJECTORY_FOLLOWING;
        trajectoryStartTime = 0;
        this.trajectory = trajectory;
        updateRobotState();
    }

    /**
     * Returns the pose of the drivetrain with respect to the field
     *
     * @return Pose2d fieldToVehicle
     */
    public Pose2d getPose() {
        return robotState.fieldToVehicle;
    }

    /**
     * Periodically updates the Trajectory based on a timestamp and the desired pose at that point which feeds into a
     * closed loop controls system, part of TRAJECTORY_FOLLOWING
     *
     * @param timestamp
     */
    public void updateTrajectoryPeriodic(double timestamp) {
        if (trajectory == null) {
            return;
        }
        if (trajectoryStartTime == 0) trajectoryStartTime = timestamp;
        // update desired pose from trajectory
        desiredPose = trajectory.sample(timestamp - trajectoryStartTime).poseMeters;
        desiredHeading = desiredPose.getRotation();
    }

    /**
     * Periodically updates drivetrain actions in open loop control
     */
    protected void updateOpenLoopPeriodic() {
    }

    /**
     * Updates the RobotState based on odometry and other calculations
     */
    protected abstract void updateRobotState();

    /**
     * Configure motors for open loop control and sends a DriveSignal command based on inputs
     *
     * @param signal DriveSignal
     * @see com.team1816.lib.util.team254.SwerveDriveSignal
     * @see DriveSignal
     */
    public abstract void setOpenLoop(DriveSignal signal);

    /**
     * Sets the tele-operated inputs for the drivetrain that will be translated into a DriveSignal
     *
     * @param forward  forward demand
     * @param strafe   strafe demand
     * @param rotation rotation demand
     * @see this#setOpenLoop(DriveSignal)
     */
    public abstract void setTeleopInputs(double forward, double strafe, double rotation);

    /**
     * Sets the control state of the drivetrain (TRAJECTORY FOLLOWING, OPEN LOOP)
     *
     * @param controlState ControlState
     */
    public void setControlState(ControlState controlState) {
        this.controlState = controlState;
    }

    /**
     * Sets the drivetrain to be in slow mode which will modify the drive signals and the motor demands
     *
     * @param slowMode (boolean) isSlowMode
     */
    public void setSlowMode(boolean slowMode) {
        isSlowMode = slowMode;
    }

    /**
     * Returns the actual heading of the drivetrain based on Odometry and gyroscopic measurements
     *
     * @return (Rotation2d) actualHeading
     */
    public synchronized Rotation2d getActualHeading() {
        return actualHeading;
    }

    /**
     * Returns the ControlState of the drivetrain
     *
     * @return controlState
     */
    public ControlState getControlState() {
        return controlState;
    }

    /**
     * Returns the PIDSlotConfiguration of the drivetrain (base pid)
     *
     * @return pidConfig
     */
    @Override
    public abstract PIDSlotConfiguration getPIDConfig();

    /**
     * Returns the desired heading of the drivetrain in degrees
     *
     * @return desiredHeading (degrees)
     */
    @Override
    public double getDesiredHeadingDegrees() {
        return desiredHeading.getDegrees();
    }

    /**
     * Returns the actual heading of the drivetrain (based on gyroscopic measurements) in degrees
     *
     * @return actualHeading (degrees)
     */
    @Override
    public double getActualHeadingDegrees() {
        return actualHeading.getDegrees();
    }

    /**
     * Returns the actual displacement of the drivetrain in the X direction (long side of the field) from its initial position
     *
     * @return xDisplacementActual
     */
    @Override
    public double getFieldXDisplacement() {
        return getPose().getX() - startingPose.getX();
    }

    /**
     * Returns the actual displacement of the drivetrain in the Y direction (short side of the field) from its initial position
     *
     * @return yDisplacementActual
     */
    @Override
    public double getFieldYDisplacement() {
        return getPose().getY() - startingPose.getY();
    }

    /**
     * Returns the desired displacement of the drivetrain in the X direction from its initial position
     *
     * @return xDisplacementDemanded
     */
    @Override
    public double getFieldDesiredXDisplacement() {
        return desiredPose.getX() - startingPose.getX();
    }

    /**
     * Returns the desired displacement of the drivetrain in the Y direction from its initial position
     *
     * @return yDisplacementDemanded
     */
    @Override
    public double getFieldDesiredYDisplacement() {
        return desiredPose.getY() - startingPose.getY();
    }

    /**
     * Returns whether the drivetrain is braking (usually only used in initialization)
     *
     * @return isBraking
     */
    public boolean isBraking() {
        return isBraking;
    }

    /**
     * Sets whether the drivetrain is braking
     *
     * @param on (boolean) braking
     */
    public abstract void setBraking(boolean on);

    /**
     * Resets the odometry calculations to a specific pose (typically used in parallel with a vision processing env)
     * to accurately re-localize position
     *
     * @param pose
     */
    public abstract void resetOdometry(Pose2d pose);

    /**
     * Returns if the drivetrain is in demoMode (slower)
     *
     * @return (boolean) isDemoMode
     */
    public boolean isDemoMode() {
        return isDemoMode;
    }

    /**
     * Zeroes the drivetrain to its "zero" state (configuration)
     */
    @Override
    public void zeroSensors() {
        zeroSensors(getPose());
    }

    /**
     * Zeroes the drivetrain to its "zero" state (configuration)
     *
     * @param pose
     * @see TankDrive#zeroSensors()
     * @see SwerveDrive#zeroSensors()
     */
    public abstract void zeroSensors(Pose2d pose);

    /**
     * Stops the drivetrain (default subsystem method)
     */
    @Override
    public abstract void stop();

    /**
     * Tests the drivetrain to perform a set of tasks
     *
     * @return true if tests passed
     */
    @Override
    public abstract boolean testSubsystem();

    /**
     * Initializes a SmartDashboard / Shuffleboard sendable builder for the Drivetrain state
     *
     * @param builder controlState
     * @see SendableBuilder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
            "Drive/ControlState",
            () -> this.getControlState().toString(),
            null
        );
    }

    /**
     * Returns the trajectory that the drivetrain is following if any
     *
     * @return trajectory
     */
    public Trajectory getTrajectory() {
        return trajectory;
    }

    /**
     * Returns the time from starting a trajectory
     *
     * @return timestampToTrajectoryStart
     */
    public synchronized double getTrajectoryTimestamp() {
        return timestamp - trajectoryStartTime;
    }

    /**
     * Simulates the gyroscope in a simulation environment only
     */
    public void simulateGyroOffset() {
        double simGyroOffset = chassisSpeed.omegaRadiansPerSecond * tickRatioPerLoop;
        gyroDrift -= 0;
        infrastructure.simulateGyro(simGyroOffset, gyroDrift);
    }

    /**
     * Enum for the ControlState
     */
    public enum ControlState {
        OPEN_LOOP, // open loop voltage control
        TRAJECTORY_FOLLOWING,
    }

    /**
     * Enum for DemoModes
     */
    private enum DemoMode {
        SLOW,
        COMFORT,
        SPORT,
        PLAID,
    }

    /**
     * Updates the demoMode to the desiredMode
     *
     * @return true if demoMode updated
     */
    public boolean update() {
        DemoMode selectedMode = demoModeChooser.getSelected();
        boolean modeChanged = desiredMode != selectedMode;

        // if auto has been changed, update selected auto mode + thread
        if (modeChanged) {
            System.out.println(
                "Demo mode changed from: " + desiredMode + ", to: " + selectedMode.name()
            );

            switch (selectedMode) {
                case SLOW:
                    demoModeMultiplier = 0.25;
                    break;
                case COMFORT:
                    demoModeMultiplier = 0.5;
                    break;
                case SPORT:
                    demoModeMultiplier = 0.75;
                    break;
                case PLAID:
                    demoModeMultiplier = 1;
                    break;
            }
        }
        desiredMode = selectedMode;

        return modeChanged;
    }
}
