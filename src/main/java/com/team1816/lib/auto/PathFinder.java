package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

/**
 * Utilizes Dijkstra's algorithm for a complete static obstacle avoidance path-finder which may be adapted for dynamic purposes
 */
public class PathFinder {
    private Translation2d target;
    private Translation2d robot;
    private List<Polygon> obstacles;
    private boolean isSwerve;

    /**
     * Initializes the PathFinder using cartesian poses of the target, robot, and grown obstacles
     * @param target - target
     * @param robot - current
     * @param obstacles - grown obstacles
     */
    public PathFinder (Translation2d target, Translation2d robot, List<Polygon> obstacles, boolean isSwerve) {
        this.target = target;
        this.robot = robot;
        this.obstacles = obstacles;

        this.isSwerve = isSwerve;
    }

    /**
     * Returns the waypoints for the optimal path from the robot to the target utilizing a visibility graph and Dijkstra's algorithm
     * @return waypoints
     */
    public List<Pose2d> getWaypoints () {
        List<Pose2d> waypoints = new ArrayList<>(); // returned for pathing
        List<Translation2d> points = new ArrayList<>(); // used for visibility graph
        List<Translation2d[]> edges = new ArrayList<>(); // used for visibility graph
        List<Double> cost = new ArrayList<>(); // used for dijkstra's algorithm
        List<Translation2d[]> shortestPath = new ArrayList<>(); // used for dijkstra's algorithm
        // Populate the points
        points.add(target);
        points.add(robot);
        for (Polygon obstacle : obstacles) {
            points.addAll(obstacle.getVertices());
        }
        // Generate the visibility graph
        for (Translation2d i: points) { // initial
            for (Translation2d f: points) { // final
                if (i.equals(f)) {
                    continue;
                } else {
                    // declares line segment as Line2D
                    boolean intersected = false;
                    Line2D edge = new Line2D.Double(i.getX(), i.getY(), f.getX(), f.getY());

                    // checks if line segment intersects the polygons
                    for (Polygon o: obstacles) {  // n^2log(n)
                        if (o.intersects(edge)) {
                            intersected = true;
                            break;
                        }
                    }
                    if (!intersected) {
                        edges.add(new Translation2d[] {i, f});
                    }
                }
            }
        }
        // Determine linear cost
        for (int i = 0; i < edges.size(); i++) {
            cost.add(edges.get(i)[0].getDistance(edges.get(i)[1]));
        }
        // Dijkstra for the shortest path

        // Append waypoints
        if (isSwerve) {

        } else { // modifies headings to look ahead

        }
        return waypoints;
    }
}
