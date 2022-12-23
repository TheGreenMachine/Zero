package com.team1816.lib.auto.paths;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxAccelMeters;
import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

public class PathUtil {

    /** Constraints */
    private static final double kMaxVelocity = kPathFollowingMaxVelMeters;
    private static final double kMaxAccel = kPathFollowingMaxAccelMeters;

    /**
     * Generates a trajectory based on a list of waypoints based on WPIlib's TrajectoryGenerator
     * @param usingApp
     * @param waypoints
     * @return trajectory
     * @see com.team1816.lib.auto.modes.AutoMode
     * @see AutoPath
     * @see edu.wpi.first.math.trajectory.TrajectoryGenerator
     */
    public static Trajectory generateTrajectory(
        boolean usingApp,
        List<Pose2d> waypoints
    ) {
        /* Inch to meter conversions for waypoints for trajectory calculations */
        List<Pose2d> waypointsMeters = new ArrayList<>();
        for (Pose2d pose2d : waypoints) {
            waypointsMeters.add(
                new Pose2d(
                    Units.inchesToMeters(pose2d.getX()),
                    Units.inchesToMeters(pose2d.getY()),
                    pose2d.getRotation()
                )
            );
        }
        /* Configures trajectory constraints */
        TrajectoryConfig config = new TrajectoryConfig(kMaxVelocity, kMaxAccel);
        var baseTrajectory = edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory(
            waypointsMeters,
            config
        );
        /* If web application is not used, then the starting pose is transformed to the default starting pose, this has no impact on how the trajectory is run */
        if (!usingApp) {
            baseTrajectory =
                baseTrajectory.transformBy(
                    new Transform2d(
                        Constants.kDefaultZeroingPose.getTranslation(),
                        Constants.kDefaultZeroingPose.getRotation()
                    )
                );
        }
        return baseTrajectory;
    }

    /**
     * Generates headings that can be transposed onto a trajectory with time calibration via a differential model
     * @param usingApp
     * @param waypoints
     * @param swerveHeadings
     * @return headings
     */
    public static List<Rotation2d> generateHeadings(
        boolean usingApp,
        List<Pose2d> waypoints,
        List<Rotation2d> swerveHeadings
    ) {
        if (waypoints == null || swerveHeadings == null) {
            return null;
        }

        double startX = .5;
        double startY = Constants.fieldCenterY;

        /* Trajectory is generated */
        Trajectory trajectory = generateTrajectory(usingApp, waypoints);
        List<Pose2d> waypointsMeters = new ArrayList<>();
        if (usingApp) {
            startX = 0;
            startY = 0;
        }
        /* Inch to meter conversions */
        for (Pose2d pose2d : waypoints) {
            waypointsMeters.add(
                new Pose2d(
                    Units.inchesToMeters(pose2d.getX()) + startX,
                    Units.inchesToMeters(pose2d.getY()) + startY,
                    pose2d.getRotation()
                )
            );
        }

        /* Time constraint calibration */
        List<Double> waypointTimes = new ArrayList<>();
        List<Integer> waypointIndexes = new ArrayList<>();
        int iWaypointCheckpoint = 0;
        for (Pose2d waypointPose2d : waypointsMeters) {
            for (int i = iWaypointCheckpoint; i < trajectory.getStates().size(); i++) {
                var trajectoryPose2d = trajectory.getStates().get(i).poseMeters;
                if (trajectoryPose2d.equals(waypointPose2d)) {
                    waypointTimes.add(trajectory.getStates().get(i).timeSeconds);
                    waypointIndexes.add(i);
                    break;
                }
                iWaypointCheckpoint++;
            }
        }

        /* list of headings generated with equivalent length to the trajectory poses */
        List<Rotation2d> generatedHeadings = new ArrayList<>();
        for (
            int nextCheckpoint = 1;
            nextCheckpoint < waypointsMeters.size();
            nextCheckpoint++
        ) {
            int iStart = waypointIndexes.get(nextCheckpoint - 1);
            int iEnd = waypointIndexes.get(nextCheckpoint);
            double totalDHeading =
                (
                    swerveHeadings.get(nextCheckpoint).getDegrees() -
                    swerveHeadings.get(nextCheckpoint - 1).getDegrees()
                );
            double timeBetweenWaypoints =
                waypointTimes.get(nextCheckpoint) - waypointTimes.get(nextCheckpoint - 1);
            double dHeading = totalDHeading / timeBetweenWaypoints;

            for (int i = iStart; i < iEnd; i++) {
                generatedHeadings.add(
                    Rotation2d.fromDegrees(
                        swerveHeadings.get(nextCheckpoint - 1).getDegrees() +
                        dHeading *
                        (
                            trajectory.getStates().get(i).timeSeconds -
                            waypointTimes.get(nextCheckpoint - 1)
                        )
                    )
                );
            }
        }

        /* Adds final heading to be the same as the previous so the path is aligned with the heading */
        generatedHeadings.add(swerveHeadings.get(swerveHeadings.size() - 1));

        return generatedHeadings;
    }
}
