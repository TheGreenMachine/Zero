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
            new Pose2d(1.7, 3.24, Rotation2d.fromDegrees(0)),
            new Pose2d(4.61, 2.85, Rotation2d.fromDegrees(0)),
            new Pose2d(6.8, 3.07, Rotation2d.fromDegrees(-30)),
            new Pose2d(3.8, 2.34, Rotation2d.fromDegrees(180))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(90)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
