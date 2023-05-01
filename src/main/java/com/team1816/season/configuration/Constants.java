package com.team1816.season.configuration;

import com.google.inject.Singleton;
import com.team1816.lib.auto.Polygon;
import com.team1816.lib.auto.Symmetry;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.season.Robot;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This class contains all constants pertinent to robot-specific aspects.
 * Only fields that are necessary and generalizable across systems belong in this class.
 */
@Singleton
public class Constants {

    /**
     * Factory & Stem
     */
    private static final RobotFactory factory = Robot.getFactory();

    public static final Pose2d EmptyPose2d = new Pose2d();
    public static final Rotation2d EmptyRotation2d = new Rotation2d();
    public static final Transform2d EmptyTransform2d = new Transform2d();

    public static final Pose3d EmptyPose3d = new Pose3d();
    public static final Rotation3d EmptyRotation3d = new Rotation3d();
    public static final Transform3d EmptyTransform3d = new Transform3d();
    public static final Quaternion EmptyQuaternion = new Quaternion();

    public static final double kLooperDt = factory.getConstant("kLooperDt", .020);

    /**
     * CAN Timeouts
     */
    public static final int kCANTimeoutMs = 10; // utility: on the fly updates
    public static final int kLongCANTimeoutMs = 100; // utility: constructors

    /**
     * Field characterization
     */
    public static final Symmetry fieldSymmetry = Symmetry.AXIS;
    public static final double fieldCenterY = 8.23 / 2.0;
    public static final double fieldCenterX = 16.46 / 2.0;
    public static final Pose2d fieldCenterPose = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation2d
    );
    public static final Pose2d targetPos = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation2d
    );
    public static final Pose2d kDefaultZeroingPose = new Pose2d(
        0.5,
        fieldCenterY,
        EmptyRotation2d
    );

    public static final Pose2d kFlippedZeroingPose = new Pose2d(
        fieldCenterX * 2 - .5,
        fieldCenterY,
        Rotation2d.fromDegrees(180)
    );

    public static final Pose2d kCameraMountingOffset = new Pose2d(
        0.20,
        0.19,
        Constants.EmptyRotation2d
    );
    public static final Translation2d kTurretMountingOffset = new Translation2d(
        -0.12065,
        0.13335
    );
    public static final double chargeStationThresholdXMinBlue = 2.4;
    public static final double chargeStationThresholdXMaxBlue = 5.1;
    public static final double chargeStationThresholdXMinRed = 11.4;
    public static final double chargeStationThresholdXMaxRed = 14.1;
    public static final double chargeStationThresholdYMin = 1.1;
    public static final double chargeStationThresholdYMax = 4.6;

    /**
     * Drivetrain characterization
     */
    public static final double gravitationalAccelerationConstant = 9.8d;
    public static double kMaxAccelDiffThreshold = 2d; // m/s^2
    public static double kMaxBalancingVelocity = 0.2; // m/s
    public static double kMinTrajectoryDistance = 0.064; // m
    public static double kMaxProximityThresholdCentimeters = 25; // cm
    public static double preTargetDistance = 0.4; // m

    /**
     * Elevator characterization
     */
    public static final double maxArmFeedForward = 0.075;

    /**
     * Camera characterization
     */
    public static final double kCameraMountingAngleY = 0; // degrees
    public static final double kTurretZedRadius = Units.inchesToMeters(7); // meters

    public static final double kLoggingDiskPartitionRatio = 0.25; // percent of storage space allotted for logging
    public static final boolean kLoggingRobot = factory.getConstant("logRobot") > 0;
    public static final boolean kLoggingDrivetrain = factory.getConstant("logDrivetrain") > 0 && kLoggingRobot;

    public static final double kBallEjectionDuration = factory.getConstant(
        "shooter",
        "ballEjectionDuration",
        1d
    );
    public static final boolean kUseVision = factory.getSubsystem("camera").implemented;

    /**
     * Auto Balance
     */
    public static final double autoBalanceThresholdDegrees = factory.getConstant("drivetrain", "autoBalanceThreshold", 2);
    public static final double autoBalanceDivider = factory.getConstant("drivetrain", "autoBalanceDivider", 30);

    /**
     * Pathfinder characterization
     */
    public static final Polygon blueChargeStation = new Polygon(
        new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMax),
        new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMax),
        new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMin),
        new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMin)
    );
    public static final Polygon redChargeStation = new Polygon(
        new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMax),
        new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMax),
        new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMin),
        new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMin)
    );

    /**
     * Simulation
     */
    public static final ShuffleboardTab kSimWindow = Shuffleboard.getTab("Simulation");
}
