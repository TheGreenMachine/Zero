package com.team1816.lib.auto.newpathingstuff.fieldobstaclestuff;

import com.team1816.lib.auto.newpathingstuff.Pixel;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

/**
 * An abstract class blueprint for creating FIRST competition field obstacles, accounting for special cases of driveability, and with intersection methods accounting for the size of the robot
 * All calculations assume a polygon object defined by only its vertices
 */
public abstract class FieldObstacle {
    /**
     * Properties
     */
    //List of coordinates in order of how the polygon(or really obstacle) is connected
    private final List<Translation2d> vertices;
    private final Translation2d robotCenter = Constants.robotCenter;
    private final double robotWidth = Constants.robotWidth;
    private final double robotLength = Constants.robotLength;
    private final double robotSizeLeeway = Constants.robotSizeLeeway;
    private final double boundingBoxLeeway = Constants.boundingBoxLeeway;
    private final double timeInSecondsIncrement = Constants.timeInSecondsIncrement;
    private final double robotHalfWidthLeft = robotCenter.getX() + robotSizeLeeway;
    private final double robotHalfWidthRight = robotWidth - robotCenter.getX() + robotSizeLeeway;
    private final double robotHalfLengthBottom = robotCenter.getY() + robotSizeLeeway;
    private final double robotHalfLengthTop = robotLength - robotCenter.getY() + robotSizeLeeway;
    private final Pixel pointInObstacle;


    public FieldObstacle(List<Translation2d> vertices){
        //TODO make sure this works with a null input, 0 points, 1 points, or 2 points
        this.vertices = vertices;
        pointInObstacle = findPointInObstacle();
    }

    /**
     * Checks if the inputted point is contained in the game obstacle polygon's area (currently only returns false)
     *
     * @param point
     * @return boolean
     */
    public boolean contains(Translation2d point) {
        //Checking if the obstacle contains input point using a simplified(not completely exact) version of the ray casting algorithm
        //Note: algorithm only needs one point on the bounding box, which is why only the 0 index is used in this method
        if(!intersects(point, getGeneralObstacleBoundingBox().get(0)))
            return false;
        //should only pass if statement if the point is somewhere inside the generalObstacleBoundingBox

        int numOfIntersects = 0;
        //now checks the number of sides intersected, if odd, then the point is in the polygon, if even, then the point is not
        //For every side on the game object, checks if it collides with the robot path except the "last" side in the list
        for (int i = 0; i < vertices.size() - 1; i++) {
            if (intersectsPathBoundingBox(vertices.get(i), vertices.get(i + 1), point, getGeneralObstacleBoundingBox().get(0)))
                numOfIntersects++;
        }
        //Checks collision with the "last" side
        if (intersectsPathBoundingBox(vertices.get(vertices.size() - 1), vertices.get(0), point, getGeneralObstacleBoundingBox().get(0)))
            numOfIntersects++;

        //returns true if number of sides intersected is odd, false if even
        return numOfIntersects % 2 == 1;
    }

    /**
     * Returns the 4 vertices of the bounding box of the game object, in quadrant order, not a tight fit box cause this should only be used for the contains method above
     *
     * @return List<Translation2d>
     */
    public List<Translation2d> getGeneralObstacleBoundingBox(){
        double maxX = Double.MIN_VALUE, minX = Double.MAX_VALUE, maxY = Double.MIN_VALUE, minY = Double.MAX_VALUE;

        //finds the minimum and maximum values of whatever, theoretically creating the bounding box values
        for (Translation2d vertex: vertices){
            maxX = Math.max(maxX, vertex.getX());
            minX = Math.max(minX, vertex.getX());
            maxY = Math.max(maxY, vertex.getY());
            minY = Math.max(minY, vertex.getY());
        }

        //literally just to avoid double inexact calculation errors, shouldn't increase calculation size
        maxX+=boundingBoxLeeway;
        minX-=boundingBoxLeeway;
        maxY+=boundingBoxLeeway;
        minY-=boundingBoxLeeway;

        return new ArrayList<>(List.of(new Translation2d(maxX, maxY), new Translation2d(minX, maxY), new Translation2d(minX, minY), new Translation2d(maxX, minY)));
    }

    /**
     * Method for checking if the trajectory of the robot will collide with this game object, taking into account the physical size of the robot
     *
     * @param trajectory length of robot
     * @return boolean
     */
    public boolean intersects(Trajectory trajectory) {
        //literally just samples the robot's path per increment in seconds to approximate the trajectory, and then runs the intersects code on it
        for (double i = 0; i < trajectory.getTotalTimeSeconds() - timeInSecondsIncrement; i += timeInSecondsIncrement) {
            if (intersects(trajectory.sample(i).poseMeters.getTranslation(), trajectory.sample(i + timeInSecondsIncrement).poseMeters.getTranslation()))
                return true;
        }

        return false;
    }

    /**
     * Method for checking if a line from p1 to p2 would intersect any side of the game object (in 2d), accounting for robot size, returns true if the robot path collides with object
     *
     * @param centerOfRobotOrigin
     * @param centerOfRobotTo
     * @return boolean
     */
    public boolean intersects(Translation2d centerOfRobotOrigin, Translation2d centerOfRobotTo) {
        //For every side on the game object, checks if it collides with the robot path except the "last" side in the list
        for (int i = 0; i < vertices.size() - 1; i++) {
            if (intersectsPathBoundingBox(vertices.get(i), vertices.get(i + 1), centerOfRobotOrigin, centerOfRobotTo))
                return true;
        }

        //Checks collision with the "last" side
        if (intersectsPathBoundingBox(vertices.get(vertices.size() - 1), vertices.get(0), centerOfRobotOrigin, centerOfRobotTo))
            return true;

        return false;
    }

    /**
     * Literally just does normal intersection calculations, but now basically with a thicker line(the size of the robot)
     *
     * @param obstacleSideBegin
     * @param obstacleSideEnd
     * @param centerOfRobotOrigin
     * @param centerOfRobotTo
     * @return
     */
    public boolean intersectsPathBoundingBox(Translation2d obstacleSideBegin, Translation2d obstacleSideEnd, Translation2d centerOfRobotOrigin, Translation2d centerOfRobotTo) {
        //Calculates the distance from centerOfRobotOrigin to centerOfRobotTo
        double pathLength = Math.sqrt(Math.pow(centerOfRobotOrigin.getX() - centerOfRobotTo.getX(), 2) + Math.pow(centerOfRobotOrigin.getY() - centerOfRobotTo.getY(), 2));

        /*
        Calculates the values of the 4 coordinates of the unrotated version of the robot's path
        This creates the bounding box of the path, centered at the origin deliberately
         */
        Translation2d topRightVertex = new Translation2d(robotHalfWidthRight, pathLength / 2 + robotHalfLengthTop);
        Translation2d topLeftVertex = new Translation2d(-robotHalfWidthLeft, pathLength / 2 + robotHalfLengthTop);
        Translation2d bottomLeftVertex = new Translation2d(-robotHalfWidthLeft, -pathLength / 2 - robotHalfLengthBottom);
        Translation2d bottomRightVertex = new Translation2d(robotHalfWidthRight, -pathLength / 2 - robotHalfLengthBottom);

        //List of the vertices of the bounding box, in quadrant order
        List<Translation2d> boundingBoxVertices = new ArrayList<Translation2d>(List.of(topRightVertex, topLeftVertex, bottomLeftVertex, bottomRightVertex));

        //Calculating the angle of the bounding box in radians
        double boundingBoxAngle = Math.acos((centerOfRobotTo.getX() - centerOfRobotOrigin.getX()) / Math.sqrt(Math.pow(centerOfRobotTo.getY() - centerOfRobotOrigin.getY(), 2) + Math.pow(centerOfRobotTo.getX() - centerOfRobotOrigin.getX(), 2)));
        //Since arccos's output radians is only 0-pi, and not 0-2pi, this makes it negative to account for the other half
        int coordinateCoefficient;
        if (centerOfRobotTo.getY() < centerOfRobotOrigin.getY())
            coordinateCoefficient = -1;
        else {
            coordinateCoefficient = 1;
        }

        //Rotates the vertices back to the original path's rotation
        boundingBoxVertices.replaceAll(translation2d -> new Translation2d(
                coordinateCoefficient * (Math.sin(boundingBoxAngle) * translation2d.getX() - Math.cos(boundingBoxAngle) * translation2d.getY()),
                coordinateCoefficient * (Math.cos(boundingBoxAngle) * translation2d.getX() + Math.sin(boundingBoxAngle) * translation2d.getY()))
        );

        //Moves the vertices back to the original location (undid the movement to the origin)
        boundingBoxVertices.replaceAll(translation2d -> new Translation2d(
                translation2d.getX() + (centerOfRobotOrigin.getX() + centerOfRobotTo.getX()) / 2,
                translation2d.getY() + (centerOfRobotOrigin.getY() + centerOfRobotTo.getY()) / 2
        ));

        /*
        checks the inputted line four times against every side of the polygon except the "last" one (the one connecting the first and last point in the vertices List)
        the four times is literally just the rectangle created from the physical size of the robot traveling along a straight path
         */
        for (int i = 0; i < boundingBoxVertices.size() - 1; i++) {
            if (Line2D.linesIntersect(
                    obstacleSideBegin.getX(), obstacleSideBegin.getY(),
                    obstacleSideEnd.getX(), obstacleSideEnd.getY(),
                    boundingBoxVertices.get(i).getX(), boundingBoxVertices.get(i).getY(),
                    boundingBoxVertices.get(i + 1).getX(), boundingBoxVertices.get(i + 1).getY())
            ) return true;
        }

        //literally just checks the line that wasn't checked in the loop (the one connecting the first and last point in the vertices List)
        if (Line2D.linesIntersect(
                obstacleSideBegin.getX(), obstacleSideBegin.getY(),
                obstacleSideEnd.getX(), obstacleSideEnd.getY(),
                boundingBoxVertices.get(boundingBoxVertices.size() - 1).getX(), boundingBoxVertices.get(boundingBoxVertices.size() - 1).getY(),
                boundingBoxVertices.get(0).getX(), boundingBoxVertices.get(0).getY())
        ) return true;

        return false;
    }

    /**
     * Returns a Pixel within the shape
     * @return Pixel
     */
    public Pixel findPointInObstacle(){
        int xShift = 0;
        ArrayList<Translation2d> boundingBox= new ArrayList<>(getGeneralObstacleBoundingBox());
        Translation2d centerOfObstacle = new Translation2d((boundingBox.get(0).getX()+boundingBox.get(2).getX())/2, (boundingBox.get(0).getY()+boundingBox.get(2).getY())/2);

        while(xShift<boundingBox.get(0).getX()-centerOfObstacle.getX()){
            if(contains(new Translation2d(centerOfObstacle.getX()+xShift, centerOfObstacle.getY())))
                return new Pixel((int)(centerOfObstacle.getX()+xShift), (int)(centerOfObstacle.getY()), 0);
            else if(contains(new Translation2d(centerOfObstacle.getX()-xShift, centerOfObstacle.getY())))
                return new Pixel((int)(centerOfObstacle.getX()-xShift), (int)(centerOfObstacle.getY()), 0);
            xShift+=1/Constants.pixelsInMeter;
        }

        //TODO log error couldn't find a containing point
        return null;
    }

    /**
     * returns the list of vertices of the game object in successive, connected side, order (in the order you think it's in)
     *
     * @return vertices
     */
    public List<Translation2d> getVertices() {
        return vertices;
    }

    public double getTimeInSecondsIncrement() {
        return timeInSecondsIncrement;
    }

    public Pixel getPointInObstacle() {
        return pointInObstacle;
    }
}