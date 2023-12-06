package com.team1816.lib.auto.newpathingstuff;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

/**
 * An abstract class blueprint for creating FIRST competition field obstacles, accounting for special cases of driveability, and with intersection methods accounting for the size of the robot
 */
public abstract class FieldObstacle {
    /**
     * Properties
     */
    //List of coordinates in order of how the polygon(or really obstacle) is connected
    private List<Translation2d> vertices;
    //TODO make this a yaml
    //btw this a mathematical thing, assuming a rectangular positioning of wheels, the center will be the midpoint of a rectangle formed by the contact point of the wheels, x is width, y is length
    private Translation2d robotCenter;
    //TODO make this a yaml
    private double robotWidth;
    //TODO make this a yaml
    private double robotLength;
    //TODO make this a yaml
    //This one isn't an exact measurement, it is still in meters but is a buffer value added to the length and width of the robot to avoid close collisions with obstacles
    private double robotSizeLeeway;
    private double robotHalfWidthLeft = robotCenter.getX()+robotSizeLeeway;
    private double robotHalfWidthRight = robotWidth-robotCenter.getX()+robotSizeLeeway;
    private double robotHalfLengthBottom = robotCenter.getY()+robotSizeLeeway;
    private double robotHalfLengthTop = robotLength-robotCenter.getY()+robotSizeLeeway;


    public FieldObstacle(List<Translation2d> vertices) {
        this.vertices = vertices;
    }

    /**
     * Checks if the inputted point is contained in the game obstacle polygon's area
     * @param point
     * @return boolean
     */
    public boolean contains(Translation2d point){
       //TODO Do this idk
       return false;
    }

    /**
     * Method for checking if the trajectory of the robot will collide with this game object (currently only returns true), taking into account the physical size of the robot
     * @param trajectory length of robot
     * @return boolean
     */
    public boolean intersects(Trajectory trajectory){
        //TODO actually figure out if we use linear interpolation for our pathing or what so that we can program this
        return true;
    }

    /**
     * Method for checking if a line from p1 to p2 would intersect any side of the game object (in 2d), accounting for robot size, returns true if the robot path collides with object
     * @param centerOfRobotOrigin
     * @param centerOfRobotTo
     * @return boolean
     */
    public boolean intersects(Translation2d centerOfRobotOrigin, Translation2d centerOfRobotTo){
        //For every side on the game object, checks if it collides with the robot path except the "last" side in the list
        for(int i = 0; i<vertices.size()-1; i++){
            if (intersectsPathBoundingBox(vertices.get(i), vertices.get(i+1), centerOfRobotOrigin, centerOfRobotTo))
                return true;
        }

        //Checks collision with the "last" side
        if (intersectsPathBoundingBox(vertices.get(vertices.size()-1), vertices.get(0), centerOfRobotOrigin, centerOfRobotTo))
            return true;

        return false;
    }

    /**
     * Literally just does normal intersection calculations, but now basically with a thicker line(the size of the robot)
      * @param startPoint
     * @param endPoint
     * @param centerOfRobotOrigin
     * @param centerOfRobotTo
     * @return
     */
    public boolean intersectsPathBoundingBox(Translation2d startPoint, Translation2d endPoint, Translation2d centerOfRobotOrigin, Translation2d centerOfRobotTo){
        //Calculates the distance from centerOfRobotOrigin to centerOfRobotTo
        double pathLength = Math.sqrt(Math.pow(centerOfRobotOrigin.getX()-centerOfRobotTo.getX(), 2)+Math.pow(centerOfRobotOrigin.getY()-centerOfRobotTo.getY(), 2));

        /*
        Calculates the values of the 4 coordinates of the unrotated version of the robot's path
        This creates the bounding box of the path, centered at the origin deliberately
         */
        Translation2d topRightVertex = new Translation2d(robotHalfWidthRight, pathLength/2+robotHalfLengthTop);
        Translation2d topLeftVertex = new Translation2d(-robotHalfWidthLeft, pathLength/2+robotHalfLengthTop);
        Translation2d bottomLeftVertex = new Translation2d(-robotHalfWidthLeft, -pathLength/2-robotHalfLengthBottom);
        Translation2d bottomRightVertex = new Translation2d(robotHalfWidthRight, -pathLength/2-robotHalfLengthBottom);

        //List of the vertices of the bounding box, in quadrant order
        List<Translation2d> boundingBoxVertices = new ArrayList<Translation2d>(List.of(topRightVertex, topLeftVertex, bottomLeftVertex, bottomRightVertex));

        //Calculating the angle of the bounding box in radians
        double boundingBoxAngle = Math.acos((centerOfRobotTo.getX()-centerOfRobotOrigin.getX())/Math.sqrt(Math.pow(centerOfRobotTo.getY()-centerOfRobotOrigin.getY(), 2)+Math.pow(centerOfRobotTo.getX()-centerOfRobotOrigin.getX(), 2)));
        //Since arccos's output radians is only 0-pi, and not 0-2pi, this makes it negative to account for the other half
        int coordinateCoefficient = 1;
        if(centerOfRobotTo.getY()<centerOfRobotOrigin.getY())
            coordinateCoefficient = -1;

        //Rotates the vertices back to the original path's rotation
        for(int i = 0; i<boundingBoxVertices.size(); i++){
            boundingBoxVertices.set(i, new Translation2d(
                    coordinateCoefficient * (Math.sin(boundingBoxAngle)*boundingBoxVertices.get(i).getX()-Math.cos(boundingBoxAngle)*boundingBoxVertices.get(i).getY()),
                    coordinateCoefficient * (Math.cos(boundingBoxAngle)*boundingBoxVertices.get(i).getX()+Math.sin(boundingBoxAngle)*boundingBoxVertices.get(i).getY()))
            );
        }

        /*
        checks the inputted line four times against every side of the polygon except the "last" one (the one connecting the first and last point in the vertices List)
        the four times is literally just the rectangle created from the physical size of the robot traveling along a straight path
         */
        for(int i = 0; i<boundingBoxVertices.size()-1; i++) {
            if (Line2D.linesIntersect(
                    startPoint.getX(), startPoint.getY(),
                    endPoint.getX(), endPoint.getY(),
                    boundingBoxVertices.get(i).getX(), boundingBoxVertices.get(i).getY(),
                    boundingBoxVertices.get(i+1).getX(), boundingBoxVertices.get(i+1).getY())
            ) return true;
        }

        //literally just checks the line that wasn't checked in the loop (the one connecting the first and last point in the vertices List)
        if (Line2D.linesIntersect(
                startPoint.getX(), startPoint.getY(),
                endPoint.getX(), endPoint.getY(),
                boundingBoxVertices.get(boundingBoxVertices.size()-1).getX(), boundingBoxVertices.get(boundingBoxVertices.size()-1).getY(),
                boundingBoxVertices.get(0).getX(), boundingBoxVertices.get(0).getY())
        ) return true;

        return false;
    }

    /**
     * returns the list of vertices of the game object in successive, connected side, order (in the order you think it's in)
     * @return vertices
     */
    public List<Translation2d> getVertices(){
        return vertices;
    }
}
