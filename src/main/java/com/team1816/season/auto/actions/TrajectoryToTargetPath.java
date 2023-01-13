package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.auto.paths.PathUtil;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryToTargetPath extends AutoPath {
    //run a trajectory using drive startTrajectory and update to run a trajectory from current robot state pose to (1, 1, 0) target

    private static RobotState robotState;
    private static Pose2d target;

    public TrajectoryToTargetPath(Pose2d pose) {
        robotState = Injector.get(RobotState.class);
        target = pose;
    }

    public TrajectoryToTargetPath() {
        robotState = Injector.get(RobotState.class);
        new TrajectoryToTargetPath(robotState.target);
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

//    @Override
//    public Trajectory getAsTrajectory() {
//        return PathUtil.generateTrajectory(usingApp(), robotState.deltaVehicle, getWaypoints());
//    }
//
//    @Override
//    public List<Rotation2d> getAsTrajectoryHeadings() {
//        return PathUtil.generateHeadings(
//            usingApp(),
//            getWaypoints(),
//            getWaypointHeadings(),
//            getAsTrajectory()
//        );
//    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
