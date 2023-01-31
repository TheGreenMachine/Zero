package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.team1816.lib.auto.Color;

import java.util.List;

public class DriveToChargePath extends AutoPath {

    public DriveToChargePath(){
    }

    public DriveToChargePath(Color color){
        super(color);
    }


    @Override
    protected List<Pose2d> getWaypoints() {
       return List.of(
           new Pose2d(1.7, 2.7, Rotation2d.fromDegrees(0)),
           new Pose2d(3.9, 2.7, Rotation2d.fromDegrees(0))
       );
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return null;
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
