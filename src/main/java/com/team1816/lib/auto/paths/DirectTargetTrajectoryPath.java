package com.team1816.lib.auto.paths;

import com.team1816.lib.Injector;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Path that seamlessly transitions from the robot's current state into a direct trajectory to specified target. Does not account for obstacles.
 */
public class DirectTargetTrajectoryPath extends AutoPath {
    public static RobotState robotState;
    private static Pose2d target;

    public DirectTargetTrajectoryPath(Pose2d pose) {
        robotState = Injector.get(RobotState.class);
        target = pose;
    }

    public DirectTargetTrajectoryPath() {
        robotState = Injector.get(RobotState.class);
        target = robotState.target;
    }

    @Override
    protected List<Pose2d> getWaypoints() { // A* accelerated path routing
        List<Pose2d> waypoints = new ArrayList<>();

        // initial waypoint in direction of target
        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getTranslation().minus(robotState.fieldToVehicle.getTranslation()).getAngle()));
        waypoints.add(target);

        return waypoints;
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        List<Rotation2d> headings = new ArrayList<>();
        var rD = robotState.fieldToVehicle.getRotation().getDegrees() - target.getRotation().getDegrees();
        if (Math.abs(rD) > rD + 360) {
            headings.add(Rotation2d.fromDegrees(robotState.fieldToVehicle.getRotation().getDegrees() + 360));
            headings.add(target.getRotation());
        } else {
            headings.add(robotState.fieldToVehicle.getRotation());
            headings.add(target.getRotation());
        }
        return headings;
    }

    @Override
    public Trajectory getAsTrajectory() {
        var translatedVelocity = new Translation2d(
            robotState.deltaVehicle.vxMetersPerSecond,
            robotState.deltaVehicle.vyMetersPerSecond).rotateBy(robotState.fieldToVehicle.getRotation().unaryMinus()
        );
        var translatedChassisSpeeds = new ChassisSpeeds(
            translatedVelocity.getX(),
            translatedVelocity.getY(),
            robotState.deltaVehicle.omegaRadiansPerSecond
        );
        return PathUtil.generateTrajectory(usingApp(), translatedChassisSpeeds, getWaypoints());
    }

    @Override
    public List<Rotation2d> getAsTrajectoryHeadings() {
        var translatedVelocity = new Translation2d(
            robotState.deltaVehicle.vxMetersPerSecond,
            robotState.deltaVehicle.vyMetersPerSecond).rotateBy(robotState.fieldToVehicle.getRotation().unaryMinus()
        );
        var translatedChassisSpeeds = new ChassisSpeeds(
            translatedVelocity.getX(),
            translatedVelocity.getY(),
            robotState.deltaVehicle.omegaRadiansPerSecond
        );
        return PathUtil.generateHeadings(
            usingApp(),
            getWaypoints(),
            getWaypointHeadings(),
            translatedChassisSpeeds
        );
    }

    @Override
    protected boolean usingApp() {
        return true;
    }
}
