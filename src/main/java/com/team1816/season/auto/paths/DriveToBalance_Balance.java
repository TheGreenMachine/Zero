package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class DriveToBalance_Balance extends AutoPath {

    public DriveToBalance_Balance() {
    }

    public DriveToBalance_Balance(Color color) {
        super(color);
    }



    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
            new Pose2d(1.7, 2.78, Rotation2d.fromDegrees(180)),
            new Pose2d(3.8, 3.5, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(180)
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
