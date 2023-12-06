package com.team1816.lib.auto.newpathingstuff;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class CircleNondriveableFieldObstacle extends FieldObstacle{
    double radius;
    public CircleNondriveableFieldObstacle(Translation2d center, double radius) {
        super(new ArrayList<Translation2d>(List.of(center)));
        this.radius = radius;
    }

    /**
     * Checks if the inputted point is contained in the game obstacle polygon's area (currently only returns false)
     *
     * @param point
     * @return boolean
     */
    @Override
    public boolean contains(Translation2d point) {
        double dist = Math.sqrt(Math.pow(point.getX()-super.getVertices().get(0).getX(), 2)+Math.pow(point.getY()-super.getVertices().get(0).getY(), 2));
        return !(dist>radius);
    }

    /**
     * Returns the 4 vertices of the bounding box of the game object, in quadrant order, not a tight fit box cause this should only be used for the contains method above
     * Edited for circle calculations
     *
     * @return List<Translation2d>
     */
    @Override
    public List<Translation2d> getGeneralObstacleBoundingBox(){
        double max = radius + 1;
        double min = radius - 1;
        return new ArrayList<>(List.of(new Translation2d(max, max), new Translation2d(min, max), new Translation2d(min, min), new Translation2d(max, min)));
    }

    /**
     * Copied over from FieldObstacle
     * @param trajectory length of robot
     * @return boolean
     */
    @Override
    public boolean intersects(Trajectory trajectory) {
        double timeInSecondsIncrement = super.getTimeInSecondsIncrement();
        //literally just samples the robot's path per increment in seconds to approximate the trajectory, and then runs the intersects code on it
        for (double i = 0; i < trajectory.getTotalTimeSeconds() - timeInSecondsIncrement; i += timeInSecondsIncrement) {
            if (intersects(trajectory.sample(i).poseMeters.getTranslation(), trajectory.sample(i + timeInSecondsIncrement).poseMeters.getTranslation()))
                return true;
        }

        return false;
    }

    /**
     * Calculating intersection of robot path with circle
     * @param centerOfRobotOrigin
     * @param centerOfRobotTo
     * @return
     */
    @Override
    public boolean intersects(Translation2d centerOfRobotOrigin, Translation2d centerOfRobotTo) {
        double twoPointFormulaA;
        double twoPointFormulaB;
        double twoPointFormulaC;

        //calculate slope of path
        double slope = (centerOfRobotOrigin.getY()-centerOfRobotTo.getY())/(centerOfRobotOrigin.getX()-centerOfRobotTo.getX());

        //calculate coefficients of the line representation of the path using two point formula
        twoPointFormulaA = centerOfRobotOrigin.getY()-centerOfRobotTo.getY();
        twoPointFormulaB = centerOfRobotOrigin.getX()-centerOfRobotTo.getX();
        twoPointFormulaC = centerOfRobotOrigin.getY()*twoPointFormulaA - centerOfRobotOrigin.getX()*twoPointFormulaB;

        //calculate the distance of the line from the center of the circle
        double distance = (Math.abs(twoPointFormulaA * super.getVertices().get(0).getX() + twoPointFormulaB  * super.getVertices().get(0).getX() + twoPointFormulaC)) / Math.sqrt(twoPointFormulaA * twoPointFormulaA + twoPointFormulaB * twoPointFormulaB);

        //if the line closer to the center of the circle than the radius of the circle, it is intersecting with the circle
        if(distance > radius)
            return false;
        else
            return true;
    }
}