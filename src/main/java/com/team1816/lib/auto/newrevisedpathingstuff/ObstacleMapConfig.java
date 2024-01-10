package com.team1816.lib.auto.newrevisedpathingstuff;

import com.team1816.season.configuration.Constants;

import java.util.ArrayList;

public class ObstacleMapConfig {
    public static UpdatableAndExpandableFieldMap makeMap(){
        //List of permanent obstacles
        int[][][] stableObstacles = {
                {{290, 540}, {290, 154}, {350, 154}, {350, 540}},
                {{450, 540}, {450, 50}, {475, 154}, {475, 390}}
        };

        //List of updatable obstacles
        int[][][] updatableObstacles = {

        };

        FieldMap stableMap = new FieldMap(Constants.xPixels, Constants.yPixels);
        FieldMap updatableMap = new FieldMap(Constants.xPixels, Constants.yPixels);

        for(int k = 0; k < stableObstacles.length; k++){
            int[] valX = new int[stableObstacles[k].length];
            int[] valY = new int[stableObstacles[k].length];

            for (int j = 0; j < stableObstacles[k].length; j++) {
                valX[j] = stableObstacles[k][j][0];
                valY[j] = stableObstacles[k][j][1];
            }

            stableMap.drawPolygon(valX, valY, true);
        }

        for(int k = 0; k < updatableObstacles.length; k++){
            int[] valX = new int[updatableObstacles[k].length];
            int[] valY = new int[updatableObstacles[k].length];

            for (int j = 0; j < updatableObstacles[k].length; j++) {
                valX[j] = updatableObstacles[k][j][0];
                valY[j] = updatableObstacles[k][j][1];
            }

            updatableMap.drawPolygon(valX, valY, true);
        }

        return new UpdatableAndExpandableFieldMap(Constants.xPixels, Constants.yPixels, stableMap, updatableMap, Constants.robotCircleFromCenterRadius);
    }
}
