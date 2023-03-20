package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.jetbrains.annotations.NotNull;

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
        Map<Translation2d, Node> nodeMap = new HashMap<>(); // used for visibility graph

        Map<Translation2d[], Double> cost = new HashMap<>(); // used for dijkstra's algorithm
        Map<Translation2d, Double> distanceHeuristic = new HashMap<>(); // used for dijkstra's algorithm
        List<Translation2d> shortestPath = new ArrayList<>(); // used for dijkstra's algorithm

        // Populates list of points and nodes
        points.add(target.getTranslation());
        Node t = new Node(target.getTranslation());
        nodeMap.put(target.getTranslation(), t);
        points.add(robot);
        Node r = new Node(robot);
        nodeMap.put(robot, r);
        for (Polygon obstacle : obstacles) {
            points.addAll(obstacle.getVertices());
            for (Translation2d v: obstacle.getVertices()) {
                Node n = new Node(v);
                nodeMap.put(v, n);
            }
        }
        // Generate the visibility graph
        for (Translation2d i : points) { // initial
            for (Translation2d f : points) { // final
                if (!i.equals(f)) {
                    // declares line segment as Line2D
                    boolean intersected = false;

                    // checks if line segment intersects the polygons
                    for (Polygon o : obstacles) {  // n^2log(n)
                        if (o.intersects(i, f)) {
                            intersected = true;
                            break;
                        }
                    }
                    if (!intersected) {
                        nodeMap.get(i).addNeighbor(nodeMap.get(f));
                    }
                }
            }
        }
        // Determine distance heuristic and neighbor cost for each node
        for (Translation2d p : nodeMap.keySet()) {
            nodeMap.get(p).calculateNeighborCost();
            nodeMap.get(p).calculateDistanceHeuristic(t);
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
        // A-star properties
        public Translation2d value;
        public double cost; // cost from start node
        public double heuristic;

        // Graph properties
        public List<Node> neighbors = new ArrayList<>();
        public Map<Node, Double> neighborCost = new HashMap<>();

        // Tree properties
        public Node previous; // null for start node at the end

        /**
         * Base constructor to initialize a node
         */
        public Node() {
        }

        /**
         * Base constructor to initialize a node
         */
        public Node(Translation2d value) {
            this.value = value;
        }

        /**
         * Base constructor to initialize a node
         */
        public Node(Translation2d value, double cost, double heuristic, ArrayList<Node> neighbors, HashMap<Node, Double> neighborCost) {
            this.value = value;
            this.cost = cost;
            this.heuristic = heuristic;
            this.neighbors = neighbors;
            this.neighborCost = neighborCost;
        }

        /**
         * Calculates cost of all neighbors based on euclidian distance metric
         */
        public void calculateNeighborCost() {
            for (Node n: neighbors) {
                neighborCost.put(n, n.value.getDistance(this.value));
            }
        }

        /**
         * Calculates euclidean distance metric to another node
         */
        public void calculateDistanceHeuristic(Node n) {
            this.heuristic = this.value.getDistance(n.value);
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
        public List<Node> getNeighbors() {
            return neighbors;
        }

        /**
         * Returns the map of the cost of each neighbor node
         *
         * @return neighborCost
         */
        public Map<Node, Double> getNeighborCost() {
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
         * @return compared value
         */
        @Override
        public int compareTo(@NotNull Object o) {
            return Double.compare(((Node) o).cost + ((Node) o).heuristic, this.cost + this.heuristic);
        }
    }
}