package com.team1816.season.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.Symmetry;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;
import java.util.List;

public class LivingRoomPath extends AutoPath {

    public LivingRoomPath() {}

    public LivingRoomPath(Color color) {
        if (Constants.fieldSymmetry == Symmetry.AXIS && color == Color.RED) {
            setReflected(true);
        } else {
            setReflected(false);
        }
    }
    @Override
    public List<Pose2d> getWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        waypoints.add(new Pose2d(0.5, 4.1, Rotation2d.fromDegrees(0)));
        waypoints.add(new Pose2d(2.0, 4.4, Rotation2d.fromDegrees(45)));
        waypoints.add(new Pose2d(4.4, 4.9, Rotation2d.fromDegrees(0)));
        return waypoints;
    }

    @Override
    public List<Rotation2d> getWaypointHeadings() {
        return List.of(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(0)
        );
    }

    @Override
    protected List<Pose2d> getReflectedWaypoints() {
//        List<Pose2d> waypoints = new ArrayList<>();
//        waypoints.add(new Pose2d(Constants.fieldCenterX*2-0.5, 4.1, Rotation2d.fromDegrees(180-0)));
//        waypoints.add(new Pose2d(Constants.fieldCenterX*2-2.0, 4.4, Rotation2d.fromDegrees(180-45)));
//        waypoints.add(new Pose2d(Constants.fieldCenterX*2-4.4, 4.9, Rotation2d.fromDegrees(180-0)));
//        return waypoints;
        List<Pose2d> waypoints = getWaypoints();
        List<Pose2d> reflectedWaypoints = new ArrayList<>();
        for(int i = 0; i < waypoints.size(); i++) {
            Pose2d waypoint = new Pose2d(2*Constants.fieldCenterX-waypoints.get(i).getX(), waypoints.get(i).getY(), Rotation2d.fromDegrees(180-waypoints.get(i).getRotation().getDegrees()));
            reflectedWaypoints.add(i, waypoint);
        }
        return reflectedWaypoints;
    }

    @Override
    protected List<Rotation2d> getReflectedWaypointHeadings() {
//        return List.of(
//            Rotation2d.fromDegrees(180-0),
//            Rotation2d.fromDegrees(180-90),
//            Rotation2d.fromDegrees(180-0)
//        );
        List<Rotation2d> waypointHeadings = getWaypointHeadings();
        List<Rotation2d> reflectedWaypointHeadings = new ArrayList<>();
        for(int i = 0; i < waypointHeadings.size(); i++) {
            var waypointRotation = Rotation2d.fromDegrees(180-waypointHeadings.get(i).getDegrees());
            reflectedWaypointHeadings.add(i, waypointRotation);
        }
        return reflectedWaypointHeadings;
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
