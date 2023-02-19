package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.auto.paths.PathUtil;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryToTargetPath extends AutoPath {

    public static RobotState robotState;
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
        if (
            (target.getY() > Constants.chargeStationThresholdYMin && target.getY() < Constants.chargeStationThresholdYMax) &&
                (robotState.fieldToVehicle.getY() > Constants.chargeStationThresholdYMin && robotState.fieldToVehicle.getY() < Constants.chargeStationThresholdYMax)
        ) { // add two waypoints around the charging station so the path is as safe as possible
            if (robotState.allianceColor == Color.RED) { // red side
                if (robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMinRed) { // rectangular minimization
                    // average the y values and determine which vertices to choose
                    double avg = (robotState.fieldToVehicle.getY() + target.getY()) / 2;
                    if (avg > (Constants.chargeStationThresholdYMin + Constants.chargeStationThresholdYMax) / 2) { // upper case
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMinRed - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMax - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                    } else { // lower case
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMinRed - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMin - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMin), target.getRotation())); // lower bounding box
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMin), target.getRotation())); // lower bounding box
                    }
                } else { // on the charge station or beyond
                    waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation())); // straight line
                }
            } else { // blue side
                if (robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMaxBlue) { // rectangular minimization
                    double avg = (robotState.fieldToVehicle.getY() + target.getY()) / 2;
                    if (avg > (Constants.chargeStationThresholdYMin + Constants.chargeStationThresholdYMax) / 2) { // upper case
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMaxBlue - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMax - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                    } else { // lower case
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMaxBlue - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMin - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMin), target.getRotation())); // lower bounding box
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMin), target.getRotation())); // lower bounding box
                    }
                } else { // on the charge station or beyond
                    waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation())); // straight line
                }
            }
        } else if (target.getY() > Constants.chargeStationThresholdYMin && target.getY() < Constants.chargeStationThresholdYMax) { // target is in the middle but robot is not, so add waypoint closer to the scoring nodes
            waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
            if (robotState.fieldToVehicle.getY() > Constants.chargeStationThresholdYMax) { // upper bounding box
                if (robotState.allianceColor == Color.RED) {
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMax), target.getRotation()));
                } else {
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMax), target.getRotation()));
                }
            } else { // lower bounding box
                if (robotState.allianceColor == Color.RED) {
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMin), target.getRotation()));
                } else {
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMin), target.getRotation()));
                }            }
        } else if (robotState.fieldToVehicle.getY() > Constants.chargeStationThresholdYMin && robotState.fieldToVehicle.getY() < Constants.chargeStationThresholdYMax) { // target is not in the middle but robot is, so add waypoint farther from the scoring nodes
            if (target.getY() > Constants.chargeStationThresholdYMax) { // upper bounding box
                if (robotState.allianceColor == Color.RED) {
                    var angle = new Rotation2d(Constants.chargeStationThresholdXMinRed - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMax - robotState.fieldToVehicle.getY());
                    waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                } else {
                    var angle = new Rotation2d(Constants.chargeStationThresholdXMaxBlue - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMax - robotState.fieldToVehicle.getY());
                    waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                }
            } else { // lower bounding box
                if (robotState.allianceColor == Color.RED) {
                    var angle = new Rotation2d(Constants.chargeStationThresholdXMinRed - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMin - robotState.fieldToVehicle.getY());
                    waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMin), target.getRotation())); // upper bounding box
                } else {
                    var angle = new Rotation2d(Constants.chargeStationThresholdXMaxBlue - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMin - robotState.fieldToVehicle.getY());
                    waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                    waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMin), target.getRotation())); // upper bounding box
                }
            }
        } else {
            waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
        }
        waypoints.add(target);
        return waypoints;
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        List<Rotation2d> headings = new ArrayList<>();
        headings.add(robotState.fieldToVehicle.getRotation());
        for (int i = 1; i < getWaypoints().size(); i++){
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
