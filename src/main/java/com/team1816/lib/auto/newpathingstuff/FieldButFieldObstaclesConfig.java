package com.team1816.lib.auto.newpathingstuff;

import java.util.List;
import java.util.ArrayList;

/**
 * Class specifying all details about the game field, in FieldObstacle
 */
public class FieldButFieldObstaclesConfig {
    //TODO prob make this a yaml
    private double fieldInsideWidth;
    //TODO prob also make this a yaml
    private double fieldInsideLength;
    private List<FieldObstacle> FieldObstacles = new ArrayList<FieldObstacle>(List.of(
        //TODO add the list of game elements here
    ));

    public double getFieldInsideWidth() {
        return fieldInsideWidth;
    }

    public double getFieldInsideLength() {
        return fieldInsideLength;
    }

    public List<FieldObstacle> getFieldObstacles() {
        return FieldObstacles;
    }
}