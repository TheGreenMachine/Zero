package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class NodeToChargeStationMiddlePath extends AutoPath {

    public NodeToChargeStationMiddlePath() {
    }

    public NodeToChargeStationMiddlePath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.7, 3.38, Rotation2d.fromDegrees(0)),
            new Pose2d(3.89, 2.51, Rotation2d.fromDegrees(0)),
            new Pose2d(5.86, 2.71, Rotation2d.fromDegrees(0)),
            new Pose2d(3.84, 2.73, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
