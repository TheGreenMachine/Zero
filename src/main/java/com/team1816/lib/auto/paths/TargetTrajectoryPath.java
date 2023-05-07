package com.team1816.lib.auto.paths;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.PathFinder;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Path that delivers the robot to a specified target with active obstacle avoidance
 */
public class TargetTrajectoryPath extends AutoPath {

    public static RobotState robotState;
    private static Pose2d target;
    private PathFinder pathFinder;

    public TargetTrajectoryPath(Pose2d pose) {
        robotState = Injector.get(RobotState.class);
        target = pose;

        pathFinder = robotState.pathFinder;
        pathFinder.setRobot(robotState.fieldToVehicle);
        pathFinder.setTarget(target);
    }

    public TargetTrajectoryPath() {
        robotState = Injector.get(RobotState.class);
        new TargetTrajectoryPath(robotState.target);

//        pathFinder = robotState.pathFinder;
//        pathFinder.setRobot(robotState.fieldToVehicle);
//        pathFinder.setTarget(target);
    }

    @Override
    protected List<Pose2d> getWaypoints() { // A* accelerated path routing
        List<Pose2d> waypoints = new ArrayList<>();

        try {
            waypoints = pathFinder.getWaypoints();
            if (waypoints.size() > 1) {
                return waypoints;
            } else {
                waypoints.clear();
            }
        } catch (Exception ignored) {
        }

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
        } else if (
            target.getY() > Constants.chargeStationThresholdYMin && target.getY() < Constants.chargeStationThresholdYMax
        ) { // target is in the middle but robot is not or path goes across the null box, so add waypoint closer to the scoring nodes
            waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
            if (robotState.fieldToVehicle.getY() > Constants.chargeStationThresholdYMax) { // upper bounding box
                if (robotState.allianceColor == Color.RED) {
                    if (!(robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMaxRed)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMax), target.getRotation()));
                    }
                } else {
                    if (!(robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMinBlue)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMax), target.getRotation()));
                    }
                }
            } else { // lower bounding box
                if (robotState.allianceColor == Color.RED) {
                    if (!(robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMaxRed)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMin), target.getRotation()));
                    }
                } else {
                    if (!(robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMinBlue)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMin), target.getRotation()));
                    }
                }
            }
        } else if (robotState.fieldToVehicle.getY() > Constants.chargeStationThresholdYMin && robotState.fieldToVehicle.getY() < Constants.chargeStationThresholdYMax) { // target is not in the middle but robot is, so add waypoint farther from the scoring nodes
            if (target.getY() > Constants.chargeStationThresholdYMax) { // upper bounding box
                if (robotState.allianceColor == Color.RED) {
                    if (robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMinRed) {
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMinRed - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMax - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                    } else if (robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMaxRed) {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation())); // initial
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, robotState.fieldToVehicle.getY()), target.getRotation())); // get off charging station
                    } else {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
                    }
                } else {
                    if (robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMaxBlue) {
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMaxBlue - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMax - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMax), target.getRotation())); // upper bounding box
                    } else if (robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMinBlue) {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation())); // initial
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, robotState.fieldToVehicle.getY()), target.getRotation())); // get off charging station
                    } else {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
                    }
                }
            } else { // lower bounding box
                if (robotState.allianceColor == Color.RED) {
                    if (robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMinRed) {
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMinRed - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMin - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinRed, Constants.chargeStationThresholdYMin), target.getRotation())); // upper bounding box
                    } else if (robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMaxRed) {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation())); // initial
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, robotState.fieldToVehicle.getY()), target.getRotation())); // get off charging station
                    } else {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
                    }
                } else {
                    if (robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMaxBlue) {
                        var angle = new Rotation2d(Constants.chargeStationThresholdXMaxBlue - robotState.fieldToVehicle.getX(), Constants.chargeStationThresholdYMin - robotState.fieldToVehicle.getY());
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), angle)); // straighter segment to the first edge
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxBlue, Constants.chargeStationThresholdYMin), target.getRotation())); // upper bounding box
                    } else if (robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMinBlue) {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation())); // initial
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, robotState.fieldToVehicle.getY()), target.getRotation())); // get off charging station
                    } else {
                        waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
                    }
                }
            }
        } else if (
            (target.getY() > Constants.chargeStationThresholdYMax && robotState.fieldToVehicle.getY() < Constants.chargeStationThresholdYMin) ||
                (target.getY() < Constants.chargeStationThresholdYMin && robotState.fieldToVehicle.getY() > Constants.chargeStationThresholdYMax)) { // target and robot are on opposite sides of the charge station
            waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
            if (robotState.fieldToVehicle.getY() > Constants.chargeStationThresholdYMax) { // upper bounding box
                if (robotState.allianceColor == Color.RED) {
                    if (!(robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMaxRed)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMax), target.getRotation()));
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMin), Rotation2d.fromDegrees(-90)));
                    }
                } else {
                    if (!(robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMinBlue)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMax), target.getRotation()));
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMin), Rotation2d.fromDegrees(-90)));
                    }
                }
            } else { // lower bounding box
                if (robotState.allianceColor == Color.RED) {
                    if (!(robotState.fieldToVehicle.getX() > Constants.chargeStationThresholdXMaxRed)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMin), target.getRotation()));
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMaxRed, Constants.chargeStationThresholdYMax), Rotation2d.fromDegrees(90)));
                    }
                } else {
                    if (!(robotState.fieldToVehicle.getX() < Constants.chargeStationThresholdXMinBlue)) {
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMin), target.getRotation()));
                        waypoints.add(new Pose2d(new Translation2d(Constants.chargeStationThresholdXMinBlue, Constants.chargeStationThresholdYMax), Rotation2d.fromDegrees(90)));
                    }
                }
            }
        } else {
            Rotation2d angle = new Translation2d(target.getX() - robotState.fieldToVehicle.getX(), target.getY() - robotState.fieldToVehicle.getY()).getAngle();
            waypoints.add(new Pose2d(robotState.fieldToVehicle.getTranslation(), target.getRotation()));
        }
        waypoints.add(target);

        return waypoints;
    }

    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        List<Rotation2d> headings = new ArrayList<>();
        headings.add(robotState.fieldToVehicle.getRotation());
        if (robotState.allianceColor == Color.BLUE && robotState.fieldToVehicle.getRotation().getDegrees() < 0) {
            for (int i = 1; i < getWaypoints().size(); i++) {
                headings.add(target.getRotation().times(-1)); // optimizes blue side wraparound
            }
        } else {
            for (int i = 1; i < getWaypoints().size(); i++) {
                headings.add(target.getRotation());
            }
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
