package com.team1816.lib.auto.newpathingstuff;

import com.team1816.lib.auto.newpathingstuff.fieldobstaclestuff.*;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import org.jetbrains.annotations.NotNull;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class FieldPixelMap {
    private static boolean[][] fieldPixelMap;
    private static boolean initialized = false;

    @Inject
    public FieldPixelMap(){
        fieldPixelMap = new boolean[Constants.xPixels][Constants.yPixels];

        //Draw FieldObstacles
        for(FieldObstacle obstacle: FieldButFieldObstaclesConfig.getFieldObstacles()){
            drawFieldObstacle(obstacle);
        }

        initialized = true;
    }

    public void updateField(){

        fieldPixelMap = new boolean[Constants.xPixels][Constants.yPixels];

        //Draw FieldObstacles
        for(FieldObstacle obstacle: FieldButFieldObstaclesConfig.getFieldObstacles()){
            FieldPixelMap.drawFieldObstacle(obstacle);
        }

        initialized = true;
    }

    public static void drawLine(Translation2d start, Translation2d end){
        //TODO program this, remember to bloat the object pixel space
        for(Pixel pixel : BresenhamBad.draw_line((int)start.getX(), (int)start.getY(), (int)end.getX(), (int)end.getY())){
            fieldPixelMap[pixel.getX()][pixel.getY()] = true;
        }
    }

    public static void drawCircle(Translation2d center, double radius){
        //TODO program this, remember to bloat the object pixel space
    }

    public static void fill(Pixel containedPixel){
        ArrayList<Pixel> pixelsToBeFilled = new ArrayList<>(List.of(containedPixel));
        while(!pixelsToBeFilled.isEmpty()){
            Pixel currentPixel = pixelsToBeFilled.remove(0);
            fieldPixelMap[currentPixel.getX()][currentPixel.getY()] = true;
            if(currentPixel.getX()<Constants.xPixels-1 && !fieldPixelMap[currentPixel.getX()+1][currentPixel.getY()])
                pixelsToBeFilled.add(new Pixel(currentPixel.getX()+1, currentPixel.getY(), 0));
            if(currentPixel.getX()>0 && !fieldPixelMap[currentPixel.getX()-1][currentPixel.getY()])
                pixelsToBeFilled.add(new Pixel(currentPixel.getX()-1, currentPixel.getY(), 0));
            if(currentPixel.getY()<Constants.yPixels-1 && !fieldPixelMap[currentPixel.getX()][currentPixel.getY()+1])
                pixelsToBeFilled.add(new Pixel(currentPixel.getX(), currentPixel.getY()+1, 0));
            if(currentPixel.getY()>0 && !fieldPixelMap[currentPixel.getX()][currentPixel.getY()-1])
                pixelsToBeFilled.add(new Pixel(currentPixel.getX(), currentPixel.getY()-1, 0));
        }
    }

    public static void expandPixels(double radius){
        //TODO program this
        for(int i = 0; i<fieldPixelMap.length; i++)
            for(int j = 0; j< fieldPixelMap[0].length; j++){
                if(fieldPixelMap[i][j]){
                    int openSides = 0;
                    int xShift = 0;
                    int yShift = 0;
                    boolean xMoved = false;
                    boolean yMoved = false;
                    if(i<fieldPixelMap.length-1 && !fieldPixelMap[i+1][j]){
                        openSides++;
                        xShift++;
                        xMoved = true;
                    }
                    if(i>0 && !fieldPixelMap[i-1][j]) {
                        openSides++;
                        xShift--;
                        xMoved = true;
                    }
                    if(j<fieldPixelMap[0].length-1 && !fieldPixelMap[i][j+1]) {
                        openSides++;
                        yShift++;
                        yMoved = true;
                    }
                    if(j>0 && !fieldPixelMap[i][j-1]) {
                        openSides++;
                        yShift--;
                        yMoved = true;
                    }
                    if(openSides == 0);
                        //Do nothing
                    else if(openSides == 1){
                        for(int counter = 1; counter<(int)((Constants.robotWidth+Constants.robotSizeLeeway)*Constants.pixelsInMeter+0.5)+1; counter++){
                            fieldPixelMap[i+(xShift*counter)][j+(yShift*counter)] = true;
                        }
                    }else if(openSides == 2 && xShift == 0 && yShift == 0) {
                        if (xMoved)
                            for (int counter = 1; counter < (int) ((Constants.robotWidth + Constants.robotSizeLeeway) * Constants.pixelsInMeter + 1) + 1; counter++) {
                                fieldPixelMap[i + counter][j] = true;
                                fieldPixelMap[i - counter][j] = true;
                            }
                        else if (yMoved)
                            for (int counter = 1; counter < (int) ((Constants.robotWidth + Constants.robotSizeLeeway) * Constants.pixelsInMeter + 1) + 1; counter++) {
                                fieldPixelMap[i][j + counter] = true;
                                fieldPixelMap[i][j - counter] = true;
                            }
                    } else{
                        drawCircle(new Translation2d(i, j), (Constants.robotWidth+Constants.robotSizeLeeway)*Constants.pixelsInMeter);
                    }
                }
            }
    }

    public static void drawFieldObstacle(FieldObstacle obstacle){
        //TODO program this, remember to fill the objects
        if(obstacle.getClass() == FieldObstacleNondriveableCircle.class){
            drawCircle(obstacle.getVertices().get(0), ((FieldObstacleNondriveableCircle) obstacle).getRadius());

        } else if(obstacle.getClass() == FieldObstacleNondriveablePolygon.class){
            for(int i = 0; i<obstacle.getVertices().size()-1; i++){
                drawLine(obstacle.getVertices().get(i), obstacle.getVertices().get(i+1));
            }
            drawLine(obstacle.getVertices().get(obstacle.getVertices().size()-1), obstacle.getVertices().get(0));
        }
        fill(obstacle.getPointInObstacle());
    }

    public static boolean checkPixel(@NotNull Pixel pixel){
        if(!initialized){
            fieldPixelMap = new boolean[Constants.xPixels][Constants.yPixels];

            //Draw FieldObstacles
            for(FieldObstacle obstacle: FieldButFieldObstaclesConfig.getFieldObstacles()){
                FieldPixelMap.drawFieldObstacle(obstacle);
            }

            initialized = true;
        }
        if(pixel.getX()<0 || pixel.getX()>fieldPixelMap.length-1 || pixel.getY()<0 || pixel.getY()>fieldPixelMap[0].length-1)
            return true;
        return fieldPixelMap[pixel.getX()][pixel.getY()];
    }
}