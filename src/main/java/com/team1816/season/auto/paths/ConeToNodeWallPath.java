package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class ConeToNodeWallPath extends AutoPath {

    public ConeToNodeWallPath() {
    }

    public ConeToNodeWallPath(Color color) {
        super(color);
    }

    @Override
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(7.2, 1.10, Rotation2d.fromDegrees(180)),
            new Pose2d(2.9, 0.61, Rotation2d.fromDegrees(180)),
            new Pose2d(1.7, 2.05, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180)
        );
    }

    @Override
    protected boolean isPrecalculated() {
        return true;
    }
}
