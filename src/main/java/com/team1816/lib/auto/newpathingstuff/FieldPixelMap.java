package com.team1816.lib.auto.newpathingstuff;

import com.team1816.lib.auto.newpathingstuff.fieldobstaclestuff.FieldObstacle;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import org.jetbrains.annotations.NotNull;

import javax.inject.Inject;
import java.util.List;

public class FieldPixelMap {
    private boolean[][] fieldPixelMap;

    @Inject
    public FieldPixelMap(){
        fieldPixelMap = new boolean[Constants.xPixels][Constants.yPixels];

        //Draw FieldObstacles
        for(FieldObstacle obstacle: FieldButFieldObstaclesConfig.getFieldObstacles()){
            drawFieldObstacle(obstacle);
        }
    }

    public void updateField(){
        //TODO program this
    }

    public void drawLine(Translation2d start, Translation2d end){
        //TODO program this, remember to bloat the object pixel space
    }

    public void drawFillPolygon(FieldObstacle obstacle){
        //TODO program this
    }

    public void drawFillCircle(FieldObstacle obstacle){
        //TODO program this
    }

    public void drawFieldObstacle(FieldObstacle obstacle){
        //TODO program this, remember to fill the objects
    }

    public boolean checkPixel(@NotNull Pixel pixel){
        if(pixel.getX()<0 || pixel.getX()>fieldPixelMap.length-1 || pixel.getY()<0 || pixel.getY()>fieldPixelMap[0].length-1)
            return true;
        return fieldPixelMap[pixel.getX()][pixel.getY()];
    }
}