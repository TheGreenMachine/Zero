package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.jetbrains.annotations.NotNull;

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
        Map<Translation2d, Double> distanceHeuristic = new HashMap<>(); // used for dijkstra's algorithm
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
            distanceHeuristic.put(points.get(i), points.get(i).getDistance(target.getTranslation()));
        }
        // Dijkstra / A* for the shortest path
        Queue<Node> openNodes = new PriorityQueue<Node>();
        // TODO: re-complete

        // Append waypoints (shortestPath in reverse)
        for (int i = shortestPath.size() - 1; i > 0; i--) {
            // look ahead to determine heading
            var pointHeading = shortestPath.get(i - 1).minus(shortestPath.get(i)).getAngle();
            waypoints.add(new Pose2d(shortestPath.get(i), pointHeading));
        }
        waypoints.add(target);
        return waypoints;
    }

    public static class Node implements Comparable {
        public Translation2d value;
        public double cost; // lowest total cost to get to this node

        public ArrayList<Node> neighbors;
        public HashMap<Node, Double> neighborCost;
        public double heuristic;


        /**
         * Calculates cost of all neighbors based on euclidian distance metric
         */
        public void calculateNeighborCost() {
            for (Node n: neighbors) {
                neighborCost.put(n, n.value.getDistance(this.value));
            }
        }

        /**
         * Returns the lowest cost to achieve the current node, 0 for start
         */
        public double getCost() {
            return cost;
        }

        /**
         * Returns the list of neighbors with respect to the current node
         *
         * @return neighbors
         */
        public ArrayList<Node> getNeighbors() {
            return neighbors;
        }

        /**
         * Returns the map of the cost of each neighbor node
         *
         * @return neighborCost
         */
        public HashMap<Node, Double> getNeighborCost() {
            return neighborCost;
        }

        /**
         * Returns the cost of a specific neighbor node
         *
         * @param n node
         * @return cost
         */
        public double getNeighborCost(Node n) {
            return neighborCost.get(n);
        }

        /**
         * Returns if the node contains a specified neighbor n
         *
         * @param n neighbor
         * @return true if neighbor exists
         */
        public boolean hasNeighbor(Node n) {
            return neighbors.contains(n);
        }

        /**
         * Ads a neighbor n to the list of neighbors associated with the node
         *
         * @param n neighbor
         */
        public void addNeighbor(Node n) {
            neighbors.add(n);
            calculateNeighborCost();
        }

        /**
         * Returns the heuristic value (distance) of the current node
         *
         * @return heuristic
         */
        public double getHeuristic() {
            return heuristic;
        }

        /**
         * Compares this node to another node, positive if favorable
         *
         * @param o other node
         * @return
         */
        @Override
        public int compareTo(@NotNull Object o) {
            return Double.compare(((Node) o).cost + ((Node) o).heuristic, this.cost + this.heuristic);
        }
    }

}
