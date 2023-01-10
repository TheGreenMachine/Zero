package com.team1816.season.configuration;

import com.google.inject.Singleton;
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

    public static final Pose2d EmptyPose = new Pose2d();
    public static final Rotation2d EmptyRotation = new Rotation2d();
    public static final Transform2d EmptyTransform = new Transform2d();

    public static final double kLooperDt = factory.getConstant("kLooperDt", .020);

    /**
     * CAN Timeouts
     */
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    /**
     * Field characterization
     */
    public static final double kTargetHeight = 104; // inches
    public static final double kTargetRadius = 26.56; // inches
    public static final double kCameraMountingHeight = 22; // inches
    public static final double kHeightFromCamToHub =
        kTargetHeight - kCameraMountingHeight; // inches
    public static final double kCameraMountingAngleY = 20; // degrees
    public static final double fieldCenterY = 8.23 / 2.0;
    public static final double fieldCenterX = 16.46 / 2.0;
    public static final Pose2d fieldCenterPose = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation
    );
    public static final Pose2d targetPos = new Pose2d(
        fieldCenterX,
        fieldCenterY,
        EmptyRotation
    );
    public static final Pose2d kDefaultZeroingPose = new Pose2d(
        0.5,
        fieldCenterY,
        EmptyRotation
    );



    /**
     * Drivetrain characterization
     */
    public static final double gravitationalAccelerationConstant = 9.8d;
    public static double kMaxAccelDiffThreshold = 2d; // m/s^2

    /**
     * Badlog
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
