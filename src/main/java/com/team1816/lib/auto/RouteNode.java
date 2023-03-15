package com.team1816.lib.auto;

import edu.wpi.first.math.geometry.Translation2d;
import org.jetbrains.annotations.NotNull;

public class RouteNode {
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

    @NotNull
    public int compareTo(@NotNull RouteNode other) {
        if (this.estimatedRemainingScore > other.estimatedRemainingScore) {
            return 1;
        } else if (this.estimatedRemainingScore < other.estimatedRemainingScore) {
            return -1;
        } else {
            return 0;
        }
    }
}