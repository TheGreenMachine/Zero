package com.team1816.season.configuration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import javax.inject.Singleton;
import java.util.HashMap;

/**
 * This Class stores drivetrain target positions relative to the field with dependence on alliance color
 */
@Singleton
public class DrivetrainTargets {
    public static HashMap<Integer, Pose2d> blueTargets = new HashMap<>() {
        {
            put(0, new Pose2d(new Translation2d(1.610, 0.681), Rotation2d.fromDegrees(180))); // farthest from red feeder station
            put(1, new Pose2d(new Translation2d(1.610, 1.132), Rotation2d.fromDegrees(180)));
            put(2, new Pose2d(new Translation2d(1.610, 1.683), Rotation2d.fromDegrees(180)));
            put(3, new Pose2d(new Translation2d(1.610, 2.296), Rotation2d.fromDegrees(180)));
            put(4, new Pose2d(new Translation2d(1.610, 2.808), Rotation2d.fromDegrees(180)));
            put(5, new Pose2d(new Translation2d(1.610, 3.402), Rotation2d.fromDegrees(180)));
            put(6, new Pose2d(new Translation2d(1.610, 4.008), Rotation2d.fromDegrees(180)));
            put(7, new Pose2d(new Translation2d(1.610, 4.482), Rotation2d.fromDegrees(180)));
            put(8, new Pose2d(new Translation2d(1.610, 4.953), Rotation2d.fromDegrees(180))); // closest to red feeder station
        }
    };
    public static HashMap<Integer, Pose2d> redTargets = new HashMap<>() {
        {
            put(0, new Pose2d(new Translation2d(14.804, 0.680), Rotation2d.fromDegrees(0))); // farthest from blue feeder station
            put(1, new Pose2d(new Translation2d(14.804, 1.132), Rotation2d.fromDegrees(0)));
            put(2, new Pose2d(new Translation2d(14.804, 1.683), Rotation2d.fromDegrees(0)));
            put(3, new Pose2d(new Translation2d(14.804, 2.296), Rotation2d.fromDegrees(0)));
            put(4, new Pose2d(new Translation2d(14.804, 2.808), Rotation2d.fromDegrees(0)));
            put(5, new Pose2d(new Translation2d(14.804, 3.402), Rotation2d.fromDegrees(0)));
            put(6, new Pose2d(new Translation2d(14.804, 4.008), Rotation2d.fromDegrees(0)));
            put(7, new Pose2d(new Translation2d(14.804, 4.482), Rotation2d.fromDegrees(0)));
            put(8, new Pose2d(new Translation2d(14.804, 4.953), Rotation2d.fromDegrees(0))); // closest to blue feeder station
        }
    };
}
