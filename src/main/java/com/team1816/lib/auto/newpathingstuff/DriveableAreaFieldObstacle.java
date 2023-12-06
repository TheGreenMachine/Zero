package com.team1816.lib.auto.newpathingstuff;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

/**
 * Wrapper class for FieldObstacle like NondriveableFieldObstacle, and exists just to have a different label
 */
public class DriveableAreaFieldObstacle extends FieldObstacle{
    public DriveableAreaFieldObstacle(List<Translation2d> vertices) {
        super(vertices);
    }
}
