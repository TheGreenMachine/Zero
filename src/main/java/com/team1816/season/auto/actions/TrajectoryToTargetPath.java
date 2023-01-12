package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryToTargetPath extends AutoPath {
    //run a trajectory using drive startTrajectory and update to run a trajectory from current robot state pose to (1, 1, 0) target

    private static RobotState robotState;
    private static Pose2d target;

    public TrajectoryToTargetPath(double x, double y, Rotation2d r){
        robotState = Injector.get(RobotState.class);
        target = new Pose2d(x, y, r);
    }

    public TrajectoryToTargetPath() {
        new TrajectoryToTargetPath(1.0, 1.0, Rotation2d.fromDegrees(180));
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(robotState.fieldToVehicle);
        waypoints.add(target);
        return waypoints;
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        List<Rotation2d> headings = new ArrayList<>();
        headings.add(robotState.fieldToVehicle.getRotation());
        headings.add(Rotation2d.fromDegrees(0));
        return headings;
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
