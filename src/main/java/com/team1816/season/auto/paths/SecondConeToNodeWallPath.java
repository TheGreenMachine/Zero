package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class SecondConeToNodeWallPath extends AutoPath {
    public SecondConeToNodeWallPath() {
    }

    public SecondConeToNodeWallPath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(6.2, 2.51, Rotation2d.fromDegrees(195)),
            new Pose2d(4.0, 0.61, Rotation2d.fromDegrees(180)),
            new Pose2d(1.7, 1.04, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(-15),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
