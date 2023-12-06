package com.team1816.lib.auto.newpathingstuff;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.awt.geom.Line2D;
import java.util.List;

/**
 * Class for creating game obstacles that can only be driven through on specific sides (ie, the charge station from ChargedUp!)
 */
public class DirectionalDriveableFieldObstacle extends FieldObstacle{
    /**
     * a list specifying which sides can be driven to, driveable index 0 maps to the line connecting vertices index 0 to vertices index 1, the last driveable boolean maps to the line connecting vertices index 0 to the last Translation2d object in vertices
     */
    private List<Boolean> driveable;

    /**
     * Constructor for creating game obstacles that can only be driven through on specific sides (ie, the charge station from ChargedUp!)
     * @param vertices
     * @param driveable (a list specifying which sides can be driven to, driveable index 0 maps to the line connecting vertices index 0 to vertices index 1, the last driveable boolean maps to the line connecting vertices index 0 to the last Translation2d object in vertices)
     */
    public DirectionalDriveableFieldObstacle(List<Translation2d> vertices, List<Boolean> driveable) {
        super(vertices);
        this.driveable = driveable;

        //TODO make sure I can even do assertions
        //to make sure that every line on the game obstacle has a corresponding driveablility boolean
        assert(vertices.size() == driveable.size());
    }

    /**
     * Method for checking if the trajectory of the robot will collide with this game object, taking into account the physical size of the robot
     * Overrides parent method bc it utilizes an overrided method from the parent class, otherwise would run wrong method(the non-overrided one)
     * Literally almost an exact copy of parent method otherwise
     * @param trajectory length of robot
     * @return boolean
     */
    @Override
    public boolean intersects(Trajectory trajectory){
        double timeInSecondsIncrement = super.getTimeInSecondsIncrement();
        //literally just samples the robot's path per increment in seconds to approximate the trajectory, and then runs the intersects code on it
        for(double i = 0; i<trajectory.getTotalTimeSeconds()-timeInSecondsIncrement; i+=timeInSecondsIncrement){
            if (intersects(trajectory.sample(i).poseMeters.getTranslation(), trajectory.sample(i+timeInSecondsIncrement).poseMeters.getTranslation()))
                return true;
        }

        return false;
    }

    /**
     * Edited version of FieldObjects's "intersects" method, now considering only the nondriveable sides of the game object
     * @param centerOfRobotOrigin
     * @param centerOfRobotTo
     * @return boolean
     */
    @Override
    public boolean intersects(Translation2d centerOfRobotOrigin, Translation2d centerOfRobotTo){
        //grabs the obstacle's "vertices" variable from FieldObstacle(the parent class)
        List<Translation2d> vertices = super.getVertices();

        //checks the inputted line against every side of the polygon except the "last" one (the one connecting the first and last point in the vertices List)
        for (int i = 0; i < vertices.size()-1; i++){
            //skips line if it is driveable, effectively making the loop only evaluate nondriveable(colliding) sides of the obstacle
            if (driveable.get(i)) continue;

            if (super.intersectsPathBoundingBox(vertices.get(i), vertices.get(i+1), centerOfRobotOrigin, centerOfRobotTo))
                return true;
        }

        /*
        literally just checks the line that wasn't checked in the loop (the one connecting the first and last point in the vertices List)
        skips the line if it is driveable
         */
        if (!driveable.get(driveable.size()-1) && super.intersectsPathBoundingBox(vertices.get(0), vertices.get(vertices.size()-1), centerOfRobotOrigin, centerOfRobotTo))
            return true;

        //theoretically should only reach here if path doesn't collide with any nondriveable sides of the obstacle
        return false;
    }

    /**
     * returns the boolean list of the game object's side's driveability(read above comments if that didn't make sense)
     * @return List<Boolean> driveable
     */
    public List<Boolean> getDriveable(){
        return driveable;
    }
}
