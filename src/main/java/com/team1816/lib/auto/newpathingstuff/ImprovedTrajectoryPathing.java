package com.team1816.lib.auto.newpathingstuff;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxAccelMeters;
import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;
import static edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory;

/**
 * Generates trajectory based off waypoints, then attempts to edit it to avoid obstacle collision
 */
public class ImprovedTrajectoryPathing {
    /**
     * Properties
     */
    private static final double timeInSecondsIncrement = Constants.timeInSecondsIncrement;
    private static FieldPixelMap fieldPixelMap = new FieldPixelMap();

    /**
     * Constraints
     */
    private static final double kMaxVelocity = kPathFollowingMaxVelMeters;
    private static final double kMaxAccel = kPathFollowingMaxAccelMeters;

    /**
     * Generates trajectory based off waypoints, then attempts to edit it to avoid obstacle collision
     *
     * @param waypoints
     * @param initial
     * @param startPose
     * @param endPose
     * @param updateField whether or not to update the playing field, only for if there are objects on the playing field which might change driveability
     * @return
     */
    public static Trajectory calculateTrajectory(final List<Translation2d> waypoints, ChassisSpeeds initial, Pose2d startPose, Pose2d endPose, boolean updateField, boolean runTests) {
        /* Configures trajectory constraints */
        TrajectoryConfig config = new TrajectoryConfig(kMaxVelocity, kMaxAccel);
        config.setStartVelocity(initial.vxMetersPerSecond);
        config.setEndVelocity(0);

        //Create max loop count
        int currentLoop = 0;
        int maxLoops = 100;

//        //Generate initial trajectory from inputs (this is the default trajectory)
//        Spline.ControlVector[] controlVectors = getCubicControlVectorsFromWaypoints(startPose, waypoints.toArray(new Translation2d[0]), endPose);
//        Trajectory trajectory = generateTrajectory(
//                controlVectors[0],
//                waypoints,
//                controlVectors[1],
//                config
//        );

        //Create an editable version of waypoints
        ArrayList<Translation2d> changedWaypoints = new ArrayList<>();
        Collections.copy(changedWaypoints, waypoints);

        a:
        while (currentLoop <= maxLoops) {
            currentLoop++;
            //Generate
            Trajectory testTrajectory = generateTrajectory(
                    startPose,
                    changedWaypoints,
                    endPose,
                    config
            );

            //creates a list of all input path waypoints except the start waypoint
            ArrayList<Translation2d> waypointChecks = new ArrayList<>();
            Collections.copy(waypointChecks, changedWaypoints);
            waypointChecks.add(endPose.getTranslation());

            HashMap<Translation2d, Double> waypointsToPathTime = new HashMap<>();

            //populates the hashmap waypointsToState
            for(Translation2d waypoint: waypointChecks){
                for(Trajectory.State state : testTrajectory.getStates()){
                    if(waypoint.equals(state.poseMeters.getTranslation()))
                        waypointsToPathTime.put(waypoint, state.timeSeconds);
                }
            }

            if(runTests)
                if(waypointsToPathTime.size() < waypointChecks.size()){}
                    //TODO log error
                else if(waypointsToPathTime.size() > waypointChecks.size()){}
                    //TODO log error

            //List in order of timeSeconds of the pixels the trajectory intercepts
            ArrayList<Pixel> testTrajectoryApproximationPixels = new ArrayList<Pixel>();

            //samples the robot's path per second increment to approximate the trajectory, and then adds it to the arraylist
            for (double i = 0; i < testTrajectory.getTotalTimeSeconds(); i += timeInSecondsIncrement) {
                //get coordinates of position in centimeters
                Translation2d testTrajectorySampledLocation = testTrajectory.sample(i).poseMeters.getTranslation().times(Constants.pixelsInMeter);

                //only adds the pixel if it's in a new location
                if (!testTrajectoryApproximationPixels.contains(new Pixel(testTrajectorySampledLocation, timeInSecondsIncrement)))
                    testTrajectoryApproximationPixels.add(new Pixel(testTrajectorySampledLocation, timeInSecondsIncrement));
            }

            //test check to make sure the pixels are ordered by time
            if(runTests)
                for (int i = 0; i < testTrajectoryApproximationPixels.size(); i++){
                    if (!(testTrajectoryApproximationPixels.get(i).getTime() < testTrajectoryApproximationPixels.get(i+1).getTime())){}
                        //TODO log the error here
                    if (testTrajectoryApproximationPixels.get(i).equals(testTrajectoryApproximationPixels.get(i+1))){}
                        //TODO log another error here
                }

            //create a pixel for
            Pixel collisionStartPixel = null;

            int currentMinPixelThreshold = 0;

            //Checks if any trajectory pixel is in the same place as an obstacle pixel
            for (Pixel pixel : testTrajectoryApproximationPixels) {
                //checks if the pixel is occupied by an obstacle
                //if so then records the previous pixel(so that we get the pixel that ISNT colliding)
                if (fieldPixelMap.checkPixel(pixel)) {
                    if(testTrajectoryApproximationPixels.indexOf(pixel) <= currentMinPixelThreshold)
                        if(runTests){}
                            //TODO log error start pixel is in an invalid position
                        else{
                            currentMinPixelThreshold++;
                            continue;
                        }

                    collisionStartPixel = testTrajectoryApproximationPixels.get(testTrajectoryApproximationPixels.indexOf(pixel)-1).copy();
                    break;
                }
            }

            //TODO could be error or could have worked
            if(collisionStartPixel == null)
                break a;

            //Trajectory.State in testTrajectory that is before the collision
            double timeBeforeCollision = -1;

            for (int i = 0; i < testTrajectory.getStates().size(); i++)
                if(testTrajectory.getStates().get(i).timeSeconds<collisionStartPixel.getTime()){
                    timeBeforeCollision = testTrajectory.getStates().get(i).timeSeconds;
                }

            if (runTests)
                if (timeBeforeCollision<0){}
                    //TODO log error

            //index of changedWaypoints to add new waypoint
            int indexAdd = -1;

            //records the index of the first input path waypoint that comes after the collision state
            for(Translation2d waypoint : waypointChecks)
                if(waypointsToPathTime.get(waypoint)>timeBeforeCollision)
                    indexAdd = waypointChecks.indexOf(waypoint);

            if(runTests)
                if(indexAdd<0){}
                    //TODO log error

            b: {
                for (int i = testTrajectoryApproximationPixels.indexOf(collisionStartPixel)+2; i < testTrajectoryApproximationPixels.size(); i++){
                    if (fieldPixelMap.checkPixel(testTrajectoryApproximationPixels.get(i))){
                        changedWaypoints.add(indexAdd, calcNewWaypoint(collisionStartPixel, testTrajectoryApproximationPixels.get(i)));
                        break b;
                    }
                }
                //TODO log error not found end of collision
            }
        }

        //TODO add log to throw an error
        return generateTrajectory(
                startPose,
                changedWaypoints,
                endPose,
                config);
    }

    public static Translation2d calcNewWaypoint(Pixel startCollision, Pixel endCollision) {
        //TODO make this

        int midpointX = (startCollision.getX()+endCollision.getX())/2;
        int midpointY = (startCollision.getY()+endCollision.getY())/2;
        Translation2d midpoint = new Translation2d((double)midpointX, (double)midpointY);

        double slopeY = endCollision.getY()-startCollision.getY();
        double slopeX = endCollision.getX()-startCollision.getX();
        double slope = slopeY/slopeX;

        double currentX = midpoint.getX();
        double currentY = midpoint.getY();

        double xInterceptAxis, xInterceptFieldTop, yInterceptAxis, yInterceptFieldRight;
        yInterceptAxis = midpoint.getY()-slope*midpoint.getX();
        xInterceptAxis = -yInterceptAxis/slope;
        yInterceptFieldRight = slope*(Constants.xPixels-1)+yInterceptAxis;
        xInterceptFieldTop = (Constants.yPixels-1-yInterceptAxis)/slope;

        Pixel maxIntercept = new Pixel(midpointX, midpointY, 0);
        double maxInterceptDistance = -1;

        if(yInterceptAxis<0 || yInterceptAxis>Constants.yPixels-1)
            if(new Translation2d(0, yInterceptAxis).getDistance(midpoint) > maxInterceptDistance){
                maxInterceptDistance = new Translation2d(0, yInterceptAxis).getDistance(midpoint);
                maxIntercept = new Pixel(0,(int)yInterceptAxis,0);
            }
        if(xInterceptAxis<0 || xInterceptAxis>Constants.xPixels-1)
            if(new Translation2d(0, xInterceptAxis).getDistance(midpoint) > maxInterceptDistance){
                maxInterceptDistance = new Translation2d(0, xInterceptAxis).getDistance(midpoint);
                maxIntercept = new Pixel((int)xInterceptAxis,0,0);
            }
        if(yInterceptFieldRight<0 || yInterceptFieldRight>Constants.yPixels-1)
            if(new Translation2d(0, yInterceptFieldRight).getDistance(midpoint) > maxInterceptDistance){
                maxInterceptDistance = new Translation2d(0, yInterceptFieldRight).getDistance(midpoint);
                maxIntercept = new Pixel(Constants.xPixels,(int)yInterceptFieldRight,0);
            }
        if(xInterceptFieldTop<0 || xInterceptFieldTop>Constants.xPixels-1)
            if(new Translation2d(0, xInterceptFieldTop).getDistance(midpoint) > maxInterceptDistance){
                maxInterceptDistance = new Translation2d(0, xInterceptFieldTop).getDistance(midpoint);
                maxIntercept = new Pixel((int)xInterceptFieldTop,Constants.yPixels,0);
            }



        for(Pixel pixel : Bresenham.draw_line(midpointX, midpointY, maxIntercept.getX(), maxIntercept.getY())){
            Pixel oppositePixel = new Pixel(midpointX-pixel.getX()+midpointX, midpointY-pixel.getY()+midpointY, 0);
            if(!FieldPixelMap.checkPixel(pixel))
                return new Translation2d(pixel.getX(), pixel.getY());
            else if(!FieldPixelMap.checkPixel(oppositePixel))
                return new Translation2d(oppositePixel.getX(), oppositePixel.getY());
        }

        //TODO log error path blocked
        return null;
    }
}