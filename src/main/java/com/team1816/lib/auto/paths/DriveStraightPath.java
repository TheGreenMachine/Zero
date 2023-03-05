package com.team1816.lib.auto.paths;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;

public class DriveStraightPath extends AutoPath {

    private final int driveDistance;

    public DriveStraightPath(int driveDistance, double maxVel) {
        this.driveDistance = driveDistance;
    }

    public DriveStraightPath(int driveDistance) {
        this(driveDistance, kPathFollowingMaxVelMeters);
    }

    public DriveStraightPath() {
        this(15);
    }

    @Override
    public List<Pose2d> getWaypoints() {
        var waypoints = List.of(
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
            new Pose2d((driveDistance), 0.0, Rotation2d.fromDegrees(0))
        );
        return waypoints;
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return null;
    }

    @Override
    protected List<Pose2d> getReflectedWaypoints() {
        var waypoints = List.of(
            new Pose2d(Constants.fieldCenterX * 2 - 0.0, 0.0, Rotation2d.fromDegrees(180)),
            new Pose2d(Constants.fieldCenterX * 2 - (driveDistance), 0.0, Rotation2d.fromDegrees(180))
        );
        return waypoints;
    }

    @Override
    protected List<Rotation2d> getReflectedWaypointHeadings() {
        return null;
    }

    @Override
    public boolean usingApp() {
        return false;
    }
}
