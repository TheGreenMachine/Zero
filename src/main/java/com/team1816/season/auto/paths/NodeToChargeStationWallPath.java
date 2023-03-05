package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class NodeToChargeStationWallPath extends AutoPath {

    public NodeToChargeStationWallPath() {
    }

    public NodeToChargeStationWallPath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.70, 0.61, Rotation2d.fromDegrees(0)),
            new Pose2d(6.76, 1.55, Rotation2d.fromDegrees(36)),
            new Pose2d(3.79, 2.32, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(90)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
