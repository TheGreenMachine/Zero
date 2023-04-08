package com.team1816.season.auto.paths;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class ChargeStationToField extends AutoPath {

    public ChargeStationToField() {
    }

    public ChargeStationToField(Color color) {
        super(color);
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return List.of(
                new Pose2d(4.75, 3.00, Rotation2d.fromDegrees(0)),
                new Pose2d(6.5, 3.00, Rotation2d.fromDegrees(0))
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
