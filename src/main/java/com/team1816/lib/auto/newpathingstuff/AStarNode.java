package com.team1816.lib.auto.newpathingstuff;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public class AStarNode {
    /**
     * Properties
     */
    private Translation2d nodeLocation;
    private Type nodeType;

    private List<AStarNode> neighbors;
    private AStarNode previousAStarNode;
    private AStarNode nextAStarNode;
    private AStarNode endAStarNode;
    private double distToPreviousNode;
    private double heuristicDistance;
    private double recordedDistance;
    private double cost = Double.MAX_VALUE;

    public AStarNode(Translation2d nodeLocation, List<AStarNode> neighbors, AStarNode previousAStarNode, AStarNode endAStarNode, Type nodeType){
        this.nodeLocation = nodeLocation;
        this.neighbors = neighbors;
        setPreviousAStarNode(previousAStarNode);
        this.endAStarNode = endAStarNode;
        this.nodeType = nodeType;
        calculateDistToPreviousNode();
        calculateHeuristicDist();
    }

    public void calculateDistToPreviousNode(){
        distToPreviousNode = nodeLocation.getDistance(previousAStarNode.nodeLocation);
    }

    public void calculateHeuristicDist(){
        heuristicDistance = nodeLocation.getDistance(endAStarNode.nodeLocation);
    }

    public void calculateRecordedDist(){
        calculateDistToPreviousNode();

        int currentLoops = 0;
        int maxLoops = 100;
        AStarNode currentAStarNode = this;
        double recordedDistance = 0;
        while(currentAStarNode.nodeType != Type.START_NODE){
            currentLoops++;
            if(currentLoops>maxLoops)
                throw new DisconnectedNetwork("Either the A* network is disconnected, or there is no AStarNode with the property START_NODE");
            recordedDistance += currentAStarNode.distToPreviousNode;
            currentAStarNode = currentAStarNode.previousAStarNode;
        }

        this.recordedDistance = recordedDistance;
    }

    public static class DisconnectedNetwork extends RuntimeException {
        /**
         * Create a new exception with the given message.
         *
         * @param message the message to pass with the exception
         */
        private DisconnectedNetwork(String message) {
            super(message);
        }
    }

    public boolean hasNeighbor(AStarNode possibleNeighbor){
        return neighbors.contains(possibleNeighbor);
    }

    public boolean addNeighbor(AStarNode neighbor){
        return neighbors.add(neighbor);
    }

    public Translation2d getNodeLocation(){
        return nodeLocation;
    }

    public boolean setNodeLocation(Translation2d nodeLocation){
        this.nodeLocation = nodeLocation;
        return true;
    }

    public List<AStarNode> getNeighbors() {
        return neighbors;
    }

    public boolean setNeighbors(List<AStarNode> neighbors){
        this.neighbors = neighbors;
        return true;
    }

    public AStarNode getPrevious(){
        return previousAStarNode;
    }

    public boolean setPreviousAStarNode(AStarNode previousAStarNode){
        this.previousAStarNode = previousAStarNode;
        this.previousAStarNode.nextAStarNode = this;
        calculateRecordedDist();
        return true;
    }

    public enum Type{
        START_NODE,

        MIDDLE_NODE,

        END_NODE
    }
}