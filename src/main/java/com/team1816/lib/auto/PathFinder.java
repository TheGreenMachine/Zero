package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Utilizes Dijkstra's algorithm for a complete static obstacle avoidance path-finder which may be adapted for dynamic purposes
 */
public class PathFinder {
    private Pose2d target;
    private Pose2d robot;
    private List<Polygon> obstacles;
    private boolean isSwerve;

    /**
     * Initializes the PathFinder using cartesian poses of the target, robot, and grown obstacles
     * @param target - target
     * @param robot - current
     * @param obstacles - grown obstacles
     */
    public PathFinder (Pose2d target, Pose2d robot, List<Polygon> obstacles, boolean isSwerve) {
        this.target = target;
        this.robot = robot;
        this.obstacles = obstacles;

        this.isSwerve = isSwerve;
    }

    /**
     * Returns the waypoints for the optimal path from the robot to the target utilizing a visibility graph
     * @return waypoints
     */
    public List<Pose2d> getWaypoints () {
        List<Pose2d> waypoints = new ArrayList<>();
        if (isSwerve) {

        } else {

        }
        return waypoints;
    }
}
