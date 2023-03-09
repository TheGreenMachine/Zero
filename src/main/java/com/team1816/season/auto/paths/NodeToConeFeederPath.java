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
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.7, 5.05, Rotation2d.fromDegrees(0)),
            new Pose2d(3.5, 4.64, Rotation2d.fromDegrees(0)),
            new Pose2d(7.2, 4.64, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(-180),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0)
        );
    }

    @Override
    protected boolean isPrecalculated() {
        return true;
    }
}
