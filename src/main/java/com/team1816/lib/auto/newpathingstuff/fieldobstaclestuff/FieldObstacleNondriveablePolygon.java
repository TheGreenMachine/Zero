package com.team1816.lib.auto.newpathingstuff.fieldobstaclestuff;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

/**
 * Wrapper class(probably, idk what that means really) for the abstract FieldObstacle class
 */
public class FieldObstacleNondriveablePolygon extends FieldObstacle {

    public FieldObstacleNondriveablePolygon(List<Translation2d> vertices) {
        super(vertices);
    }
}
