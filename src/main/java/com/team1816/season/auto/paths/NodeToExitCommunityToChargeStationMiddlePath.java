package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class NodeToExitCommunityToChargeStationMiddlePath extends AutoPath {

    public NodeToExitCommunityToChargeStationMiddlePath() {
    }

    public NodeToExitCommunityToChargeStationMiddlePath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.70, 3.24, Rotation2d.fromDegrees(0)),
            new Pose2d(3.92, 2.61, Rotation2d.fromDegrees(0)),
            new Pose2d(6.92, 2.61, Rotation2d.fromDegrees(0)),
            new Pose2d(4.20, 2.95, Rotation2d.fromDegrees(180))
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
