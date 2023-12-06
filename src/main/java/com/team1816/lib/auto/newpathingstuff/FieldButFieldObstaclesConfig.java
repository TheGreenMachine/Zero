package com.team1816.lib.auto.newpathingstuff;

import java.util.List;
import java.util.ArrayList;

/**
 * Class specifying all details about the game field, in FieldObstacle
 */
public class FieldButFieldObstaclesConfig {
    //TODO prob make this a yaml and final
    private static double fieldInsideWidth;
    //TODO prob also make this a yaml and final
    private static double fieldInsideLength;
    private static final List<FieldObstacle> FieldObstacles = new ArrayList<FieldObstacle>(List.of(
        //TODO add the list of game elements here
    ));

    public static double getFieldInsideWidth() {
        return fieldInsideWidth;
    }

    public static double getFieldInsideLength() {
        return fieldInsideLength;
    }

    public static List<FieldObstacle> getFieldObstacles() {
        return FieldObstacles;
    }
}