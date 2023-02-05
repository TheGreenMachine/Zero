package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
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
        this(12);
    }

    @Override
    public List<Pose2d> getWaypoints() {
        var waypoints = List.of(
            new Pose2d(0.0, 4.0, Rotation2d.fromDegrees(0)),
            new Pose2d((driveDistance), 4.0, Rotation2d.fromDegrees(0))
        );
        return waypoints;
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        var headings = List.of(
            Constants.EmptyRotation2d,
            Constants.EmptyRotation2d
        );
        return headings;
    }

    @Override
    public boolean isPrecalculated() {
        return true;
    }
}
