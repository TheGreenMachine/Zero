package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.awt.geom.Line2D;
import java.util.*;

/**
 * Utilizes Dijkstra's algorithm for a complete static obstacle avoidance path-finder which may be adapted for dynamic purposes
 */
public class PathFinder {
    private Pose2d target;
    private Translation2d robot;
    private List<Polygon> obstacles;

    /**
     * Initializes the PathFinder using cartesian poses of the target, robot, and grown obstacles
     *
     * @param target    - target
     * @param robot     - current
     * @param obstacles - grown obstacles
     */
    public PathFinder(Pose2d target, Translation2d robot, List<Polygon> obstacles) {
        this.target = target;
        this.robot = robot;
        this.obstacles = obstacles;
    }

    /**
     * Returns the waypoints for the optimal path from the robot to the target utilizing a visibility graph and Dijkstra's algorithm
     *
     * @return waypoints
     */
    public List<Pose2d> getWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>(); // returned for pathing

        List<Translation2d> points = new ArrayList<>(); // used for visibility graph
        List<Translation2d[]> edges = new ArrayList<>(); // used for visibility graph

        Map<Translation2d[], Double> cost = new HashMap<>(); // used for dijkstra's algorithm
        Map<Translation2d, Double> distance = new HashMap<>(); // used for dijkstra's algorithm
        List<Translation2d> shortestPath = new ArrayList<>(); // used for dijkstra's algorithm

        // Populate the points
        points.add(target.getTranslation());
        points.add(robot);
        for (Polygon obstacle : obstacles) {
            points.addAll(obstacle.getVertices());
        }
        // Generate the visibility graph
        for (Translation2d i : points) { // initial
            for (Translation2d f : points) { // final
                if (i.equals(f)) {
                    continue;
                } else {
                    // declares line segment as Line2D
                    boolean intersected = false;
                    Line2D edge = new Line2D.Double(i.getX(), i.getY(), f.getX(), f.getY());

                    // checks if line segment intersects the polygons
                    for (Polygon o : obstacles) {  // n^2log(n)
                        if (o.intersects(edge)) {
                            intersected = true;
                            break;
                        }
                    }
                    if (!intersected) {
                        edges.add(new Translation2d[]{i, f});
                    }
                }
            }
        }
        // Determine linear cost
        for (int i = 0; i < edges.size(); i++) {
            cost.put(edges.get(i), edges.get(i)[0].getDistance(edges.get(i)[1]));
        }
        // Determine distance heuristic
        for (int i = 0; i < points.size(); i++) {
            distance.put(points.get(i), points.get(i).getDistance(target.getTranslation()));
        }
        // Dijkstra / A* for the shortest path
        Queue<RouteNode> openNodes = new PriorityQueue<>();
        Map<Translation2d, RouteNode> nodes = new HashMap<>();
        RouteNode init = new RouteNode(robot, null, 0, distance.get(robot));
        openNodes.add(init);
        nodes.put(robot, init);

        while (!openNodes.isEmpty()) {
            RouteNode next = openNodes.poll();
            // backtrack to determine path if destination reached
            if (next.getCurrent().equals(target)) {
                RouteNode current = next;
                do {
                    shortestPath.add(0, current.getCurrent());
                    current = nodes.get(current.getPrevious());
                } while (current != null); // starter init block break
                break;
            }
            // otherwise continue
            for (Translation2d nextSweptNode : nodes.keySet()) {
                // sweep for next node from current head
                if (nextSweptNode.equals(next.getCurrent())) {
                    double newScore = next.getRouteScore() + nextSweptNode.getDistance(next.getCurrent());
                    RouteNode nextNode = nodes.getOrDefault(nextSweptNode, new RouteNode(nextSweptNode));
                    nodes.put(nextSweptNode, nextNode);

                    if (nextNode.getRouteScore() > newScore) {
                        nextNode.setPrevious(next.getCurrent());
                        nextNode.setRouteScore(newScore);
                        nextNode.setEstimatedRemainingScore(newScore + distance.get(nextSweptNode));

                        // add next node to priority queue
                        openNodes.add(nextNode);
                    }
                }
            }
        }

        // Append waypoints (shortestPath in reverse)
        for (int i = shortestPath.size() - 1; i > 0; i--) {
            // look ahead to determine heading
            var pointHeading = shortestPath.get(i - 1).minus(shortestPath.get(i)).getAngle();
            waypoints.add(new Pose2d(shortestPath.get(i), pointHeading));
        }
        waypoints.add(target);
        return waypoints;
    }

    public static class RouteNode {
        private final Translation2d current;
        private Translation2d previous;
        private double routeScore;
        private double estimatedRemainingScore;

        RouteNode(Translation2d current) {
            this(current, null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        RouteNode(Translation2d current, Translation2d previous, double routeScore, double estimatedRemainingScore) {
            this.current = current;
            this.previous = previous;
            this.routeScore = routeScore;
            this.estimatedRemainingScore = estimatedRemainingScore;
        }

        public Translation2d getCurrent() {
            return current;
        }

        public Translation2d getPrevious() {
            return previous;
        }

        public double getRouteScore() {
            return routeScore;
        }

        public double getEstimatedRemainingScore() {
            return estimatedRemainingScore;
        }

        public void setPrevious(Translation2d previous) {
            this.previous = previous;
        }

        public void setRouteScore(double routeScore) {
            this.routeScore = routeScore;
        }

        public void setEstimatedRemainingScore(double estimatedRemainingScore) {
            this.estimatedRemainingScore = estimatedRemainingScore;
        }

        public int compareTo(RouteNode other) {
            if (this.estimatedRemainingScore > other.estimatedRemainingScore) {
                return 1;
            } else if (this.estimatedRemainingScore < other.estimatedRemainingScore) {
                return -1;
            } else {
                return 0;
            }
        }
    }
}
