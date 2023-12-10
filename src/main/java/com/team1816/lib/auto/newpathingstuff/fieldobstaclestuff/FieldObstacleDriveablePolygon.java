package com.team1816.lib.auto.newpathingstuff.fieldobstaclestuff;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

/**
 * Wrapper class for FieldObstacle like NondriveableFieldObstacle, and exists just to have a different label
 */
public class FieldObstacleDriveablePolygon extends FieldObstacle {
    public FieldObstacleDriveablePolygon(List<Translation2d> vertices) {
        super(vertices);
    }
}
