package com.team1816.lib.auto.newpathingstuff;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

/**
 * Wrapper class(probably, idk what that means really) for the abstract FieldObstacle class
 */
public class PolygonNondriveableFieldObstacle extends FieldObstacle{

    public PolygonNondriveableFieldObstacle(List<Translation2d> vertices) {
        super(vertices);
    }
}
