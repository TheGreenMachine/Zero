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
public class

DrivetrainTargets {
    public static HashMap<Integer, Pose2d> blueTargets = new HashMap<>() {
        {
            // assumes grid horizontal of 54.25 inches and robot center of 15.5 inches
            put(0, new Pose2d(new Translation2d(2.072, 0.513), Rotation2d.fromDegrees(180))); // farthest from red feeder station
            put(1, new Pose2d(new Translation2d(2.072, 1.072), Rotation2d.fromDegrees(180)));
            put(2, new Pose2d(new Translation2d(2.072, 1.630), Rotation2d.fromDegrees(180)));
            put(3, new Pose2d(new Translation2d(2.072, 2.189), Rotation2d.fromDegrees(180)));
            put(4, new Pose2d(new Translation2d(2.072, 2.748), Rotation2d.fromDegrees(180)));
            put(5, new Pose2d(new Translation2d(2.072, 3.306), Rotation2d.fromDegrees(180)));
            put(6, new Pose2d(new Translation2d(2.072, 3.865), Rotation2d.fromDegrees(180)));
            put(7, new Pose2d(new Translation2d(2.072, 4.424), Rotation2d.fromDegrees(180)));
            put(8, new Pose2d(new Translation2d(2.072, 4.982), Rotation2d.fromDegrees(180))); // closest to red feeder station
        }
    };
    public static HashMap<Integer, Pose2d> redTargets = new HashMap<>() {
        {
            put(0, new Pose2d(new Translation2d(14.388, 0.513), Rotation2d.fromDegrees(0))); // farthest from blue feeder station
            put(1, new Pose2d(new Translation2d(14.388, 1.072), Rotation2d.fromDegrees(0)));
            put(2, new Pose2d(new Translation2d(14.388, 1.630), Rotation2d.fromDegrees(0)));
            put(3, new Pose2d(new Translation2d(14.388, 2.180), Rotation2d.fromDegrees(0)));
            put(4, new Pose2d(new Translation2d(14.388, 2.748), Rotation2d.fromDegrees(0)));
            put(5, new Pose2d(new Translation2d(14.388, 3.306), Rotation2d.fromDegrees(0)));
            put(6, new Pose2d(new Translation2d(14.388, 3.865), Rotation2d.fromDegrees(0)));
            put(7, new Pose2d(new Translation2d(14.388, 4.424), Rotation2d.fromDegrees(0)));
            put(8, new Pose2d(new Translation2d(14.388, 5.111), Rotation2d.fromDegrees(0))); // closest to blue feeder station
        }
    };
}
