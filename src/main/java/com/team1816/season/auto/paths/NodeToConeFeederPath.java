package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class NodeToConeFeederPath extends AutoPath {

    public NodeToConeFeederPath() {
    }

    public NodeToConeFeederPath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.75, 5.05, Rotation2d.fromDegrees(0)),
            new Pose2d(2.8, 4.8, Rotation2d.fromDegrees(0)),
            new Pose2d(4.5, 4.8, Rotation2d.fromDegrees(0)),
            new Pose2d(6.2, 4.7, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(40),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
