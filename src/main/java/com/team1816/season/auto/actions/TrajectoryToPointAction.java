package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryToPointAction extends AutoPath {
    //run a trajectory using drive startTrajectory and update to run a trajectory from current robot state pose to (1, 1, 0) target

    private static RobotState robotState;

    private static Rotation2d targetRotation;
    private static Pose2d target;

    public TrajectoryToPointAction(double x, double y, Rotation2d r){
        target = new Pose2d(x, y, r);
        //target = new Pose2d (1.0, 1.0, 0.0);
    }

    public TrajectoryToPointAction() {

    }

    @Override
    protected List<Pose2d> getWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)));
        waypoints.add(target);
        return waypoints;
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        List<Rotation2d> headings = new ArrayList<>();
        headings.add(Rotation2d.fromDegrees(0));
        return headings;
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
