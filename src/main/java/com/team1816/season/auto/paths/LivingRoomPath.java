package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class LivingRoomPath extends AutoPath {

    @Override
    public List<Pose2d> getWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d((79.5), (11.0), Rotation2d.fromDegrees(45)));
        waypoints.add(new Pose2d((172), (30), Rotation2d.fromDegrees(0)));
        return waypoints;
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(0)
        );
    }

    @Override
    public boolean usingApp() {
        return false;
    }
}
