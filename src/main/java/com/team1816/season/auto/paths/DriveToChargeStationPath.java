package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class DriveToChargeStationPath extends AutoPath {

    public DriveToChargeStationPath() {
    }

    public DriveToChargeStationPath(Color color) {
        super(color);
    }

    @Override
    public List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.7, 3.38, Rotation2d.fromDegrees(0)),
            new Pose2d(3.9, 2.78, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180)
        );
    }
}
