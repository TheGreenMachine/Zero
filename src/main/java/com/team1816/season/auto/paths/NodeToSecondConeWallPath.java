package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class NodeToSecondConeWallPath extends AutoPath {

    public NodeToSecondConeWallPath() {
    }

    public NodeToSecondConeWallPath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.7, 2.05, Rotation2d.fromDegrees(0)),
            new Pose2d(2.5, 0.61, Rotation2d.fromDegrees(0)),
            new Pose2d(4.5, 0.61, Rotation2d.fromDegrees(0)),
            new Pose2d(6.2, 2.51, Rotation2d.fromDegrees(15))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(15)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
