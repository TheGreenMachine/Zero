package com.team1816.season.auto.paths;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class DriveStraightPath extends AutoPath {

    private final int driveDistance;

    public DriveStraightPath(int driveDistance, double maxVel) {
        this.driveDistance = driveDistance;
    }

    public DriveStraightPath(int driveDistance) {
        this(driveDistance, kPathFollowingMaxVelMeters);
    }

    public DriveStraightPath() {
        this(12);
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
    public boolean usingApp() {
        return false;
    }
}
