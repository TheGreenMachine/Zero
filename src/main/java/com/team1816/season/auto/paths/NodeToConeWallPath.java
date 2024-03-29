package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class NodeToConeWallPath extends AutoPath {

    public NodeToConeWallPath() {
    }

    public NodeToConeWallPath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.7, 0.61, Rotation2d.fromDegrees(60.5)),
            new Pose2d(2.2, 1.10, Rotation2d.fromDegrees(12.5)),
            new Pose2d(3.2, 1.30, Rotation2d.fromDegrees(0)),
            new Pose2d(6.2, 1.30, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(-180),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
