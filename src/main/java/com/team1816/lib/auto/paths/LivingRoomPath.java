package com.team1816.lib.auto.paths;

import com.team1816.lib.auto.Color;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

public class LivingRoomPath extends AutoPath {

    public LivingRoomPath() {
    }

    public LivingRoomPath(Color color) {
        super(color);
    }

    @Override
    public List<Pose2d> getWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(2.0, 4.4, Rotation2d.fromDegrees(45)));
        waypoints.add(new Pose2d(5.4, 4.9, Rotation2d.fromDegrees(0)));
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
        return true;
    }
}
