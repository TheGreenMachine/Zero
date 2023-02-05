package com.team1816.season.configuration;

import com.google.inject.Singleton;
import com.team1816.lib.auto.Symmetry;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.season.Robot;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

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
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

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

    public static final Translation2d kTurretMountingOffset = new Translation2d(
        -.12065,
        .13335
    );

    /**
     * Drivetrain characterization
     */
    public static final double gravitationalAccelerationConstant = 9.8d;
    public static double kMaxAccelDiffThreshold = 2d; // m/s^2
    public static double kMaxBalancingVelocity = 0.2; // m/s
    public static double kMinTrajectoryDistance = 0.1; // m
    public static double kMaxProximityThresholdCentimeters = 25; // cm

    /**
     * Camera characterization
     */
    public static final double kCameraMountingAngleY = 20; // degrees
    public static final double kTurretZedRadius = Units.inchesToMeters(7); // meters

    /**
     * Badlog characterization
     */
    public static boolean kIsBadlogEnabled = factory.getConstant("badLogEnabled") > 0;
    public static boolean kIsLoggingTeleOp = factory.getConstant("logTeleOp") > 0;
    public static boolean kIsLoggingAutonomous = factory.getConstant("logAuto") > 0;

    public final boolean kUsePoseTrack =
        factory.getConstant("shooter", "usingPoseForSpeed", 0) > 0;
    public static final double kBallEjectionDuration = factory.getConstant(
        "shooter",
        "ballEjectionDuration",
        1d
    );
    public static final boolean kUseVision = factory.getSubsystem("camera").implemented;
}
