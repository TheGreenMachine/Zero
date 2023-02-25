package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class NodeToChargeStationFeederPath extends AutoPath {

    public NodeToChargeStationFeederPath() {
    }

    public NodeToChargeStationFeederPath(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.7, 4.48, Rotation2d.fromDegrees(0)),
            new Pose2d(4.09, 4.77, Rotation2d.fromDegrees(0)),
            new Pose2d(5.41, 2.78, Rotation2d.fromDegrees(180)),
            new Pose2d(3.86, 2.59, Rotation2d.fromDegrees(180))
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