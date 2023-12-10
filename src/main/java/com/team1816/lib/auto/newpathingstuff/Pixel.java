package com.team1816.lib.auto.newpathingstuff;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Stores the (x,y) value of a single pixel, x counted from 0 from the left, y counted from 0 from the top of the robots path
 * also contains a double of the time that the robot reaches this pixel
 */
public class Pixel {
    /**
     * Properties
     */
    private int x;
    private int y;
    private double time;

    /**
     * Sets values of the pixel
     * @param x
     * @param y
     * @param cartesian if true, then y is counted from the bottom, otherwise from the top
     */
    public Pixel(int x, int y, double time, boolean cartesian){
        this.x = x;
        this.y = cartesian ? Constants.yPixels-y-1 : y;
        this.time = time;
    }

    /**
     * Sets the value of the pixel using a point on the field, assumes cartesian coordinate
     * @param point
     */
    public Pixel(Translation2d point, double time){
        this.x = (int)point.getX();
        this.y = (int)point.getY();
        this.time = time;
    }

    public Pixel copy(){
        return new Pixel (x, y, time, false);
    }

    //this might not do what I want it to
    public boolean equals(Pixel pixel){
        return pixel.x == x && pixel.y == y;
    }

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

    public double getTime() {
        return time;
    }
}