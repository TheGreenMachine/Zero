package com.team1816.season.auto.actions;

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
    private static Drive drivetrain;

    private static RobotState robotState;

    private static TrajectoryAction trajectory;

    private static List<Rotation2d> headings;

    private static Pose2d target;

    private static Rotation2d targetRotation;

    private static List<Pose2d> waypoints = new ArrayList<>();

    /*private static List<Rotation2d> rotationpoints = new ArrayList<>();*/

    public TrajectoryToPointAction(double x, double y, Rotation2d r){
        trajectory = new Trajectory;
        headings = getAsTrajectoryHeadings();
        targetRotation = r;
        target = new Pose2d(x, y, targetRotation);
        //target = new Pose2d (1.0, 1.0, 0.0);
    }

    public void setWaypoints(){
        waypoints.add(robotState.fieldToVehicle);
        waypoints.add(target);
    }

    /*public List<Pose2d> getPosePoints(){
        posepoints.add(new Pose2d(50, 50, Rotation2d.fromDegrees(45)));
        posepoints.add(new Pose2d(1,1,Rotation2d.fromDegrees(0)));
        return posepoints;
    }

    public List<Rotation2d> getRotationPoints(){
        rotationpoints.add()
    }*/

    public void runTrajectoryToPointAction(){
        drivetrain.startTrajectory(trajectory, headings);
        while (drivetrain.getPose() != target){
            drivetrain.updateTrajectoryPeriodic(trajectory.getTotalTimeSeconds());
        }
    }

    @Override
    protected List<Pose2d> getWaypoints() {
        return waypoints;
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        return headings;
    }

    @Override
    protected boolean usingApp() {
        return false;
    }
}
