package com.team1816.lib.auto.newrevisedpathingstuff;

import com.team1816.lib.auto.newpathingstuff.Pixel;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import java.io.File;
import java.io.FileWriter;
import java.util.*;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxAccelMeters;
import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;
import static edu.wpi.first.math.trajectory.TrajectoryGenerator.generateTrajectory;
import static edu.wpi.first.math.trajectory.TrajectoryGenerator.splinePointsFromSplines;

public class BetterTrajectoryPathing{
    /**
     * Properties
     */
    private static final double timeInSecondsIncrement = Constants.timeInSecondsIncrement;
    private static final UpdatableAndExpandableFieldMap obstacleMap = ObstacleMapConfig.makeMap();

    /**
     * Constraints
     */
    private static final double kMaxVelocity = kPathFollowingMaxVelMeters;
    private static final double kMaxAccel = kPathFollowingMaxAccelMeters;

    /**
     * Generates trajectory based off waypoints, then attempts to edit it to avoid obstacle collision
     *
     * @param waypoints
     * @param config
     * @param startPose
     * @param endPose
     * @param updateField whether or not to update the playing field, only for if there are objects on the playing field which might change driveability
     * @return
     */
    public static Trajectory calculateTrajectory(final List<Translation2d> waypoints, TrajectoryConfig config, Pose2d startPose, Pose2d endPose, boolean updateField, boolean runTests) {
        //Create max loop count
        int currentLoop = 0;
        int maxLoops = 1000;

        //Create radius stuff
        double startMinRadius = Constants.startMinRadius;

        //Create an editable version of waypoints
        ArrayList<Translation2d> changedWaypoints = new ArrayList<>();
        if(waypoints != null)
            Collections.copy(changedWaypoints, waypoints);

        a:
        while (currentLoop <= maxLoops) {
            currentLoop++;
            //Generate
            Trajectory trajectory = generateTrajectory(
                    startPose,
                    changedWaypoints,
                    endPose,
                    config
            );

            //creates a list of all input path waypoints except the start waypoint
            ArrayList<Translation2d> waypointChecks = new ArrayList<>(changedWaypoints);
            waypointChecks.add(endPose.getTranslation());

            HashMap<Translation2d, Double> waypointsToPathTime = new HashMap<>();

            //populates the hashmap waypointsToPathTime
            for (Translation2d waypoint : waypointChecks) {
                for (Trajectory.State state : trajectory.getStates()) {
                    if (waypoint.equals(state.poseMeters.getTranslation())) {
                        if (runTests)
                            if (waypointsToPathTime.containsKey(waypoint))
                                System.out.println("Bad Things Happened");
                        waypointsToPathTime.put(waypoint, state.timeSeconds);
                    }
                }
            }

            if (waypoints != null && waypointsToPathTime.size() != waypoints.size() + 1) {
                System.out.println("Error and stuff IG");
                for(int i = 0; i < waypoints.size(); i++){
                    if(!waypointsToPathTime.containsKey(waypoints.get(i))) {
                        waypoints.remove(i);
                        i--;
                    }
                }
            }

            //List in order of timeSeconds of the pixel locations the trajectory intercepts
            ArrayList<int[]> trajectoryApproximation = new ArrayList<>();
            ArrayList<Double> trajectoryApproximationTime = new ArrayList<>();

            //samples the robot's path per second increment to approximate the trajectory, and then adds it to the arraylist
            for (double i = 0; i < trajectory.getTotalTimeSeconds(); i += timeInSecondsIncrement) {
                //get coordinates of position in centimeters
                Trajectory.State trajectorySample = trajectory.sample(i);
                Translation2d trajectorySampleLocation = trajectorySample.poseMeters.getTranslation().times(Constants.pixelsInMeter);

                //only adds the pixel if it's in a new location
                if (!trajectoryApproximation.contains(new int[]{(int)trajectorySampleLocation.getX(),(int)trajectorySampleLocation.getY()})) {
                    trajectoryApproximation.add(new int[]{(int) trajectorySampleLocation.getX(), (int) trajectorySampleLocation.getY()});
                    trajectoryApproximationTime.add(trajectorySample.timeSeconds);
                }
            }

            //test check to make sure the pixels are ordered by time
            if(runTests) {
                for (int i = 0; i < trajectoryApproximation.size()-1; i++) {
                    if (!(trajectoryApproximationTime.get(i) < trajectoryApproximationTime.get(i + 1))) {}
                    //TODO log the error here
                }
                assert(trajectoryApproximation.size() == trajectoryApproximationTime.size());
            }

            //the location of the first collision
            int[] collisionStart = null;
            //the time of the first collision
            double collisionStartTime = -1;

            //Distinguishing between error and a working loop
            boolean pathWorked = false;

            //Checks if any trajectory pixel is in the same place as an obstacle pixel
            b:
            {
                for (int i = 0; i < trajectoryApproximation.size(); i++) {
                    //checks if the pixel is occupied by an obstacle
                    //if so then records the previous pixel(so that we get the pixel that ISNT colliding)
                    if (obstacleMap.getCurrentMap().checkPixelHasObjectOrOffMap(trajectoryApproximation.get(i))) {
                        if(i < 1);
                            //TODO log error here
                        else {
                            collisionStart = trajectoryApproximation.get(i - 1);
                            collisionStartTime = trajectoryApproximationTime.get(i-1);
                            break b;
                        }
                    }
                }
                //Should've only gotten to here if the path collides with nothing
                pathWorked = true;
            }

            if(pathWorked)
                break a;

            if (runTests)
                if (collisionStartTime < 0){}
            //TODO log error

            //Trajectory.State in testTrajectory that is before the collision
            double timeBeforeCollision = -1;

            for (int i = trajectoryApproximationTime.size()-1; i >= 0 ; i--)
                if(trajectoryApproximationTime.get(i) < collisionStartTime){
                    timeBeforeCollision = trajectoryApproximationTime.get(i);
                    break;
                }

            if (runTests)
                if (timeBeforeCollision < 0){}
            //TODO log error

            //index of changedWaypoints to add new waypoint
            int indexAdd = -1;

            //records the index of the first input path waypoint that comes after the collision state
            for(Translation2d waypoint : waypointChecks)
                if(waypointsToPathTime.get(waypoint) > timeBeforeCollision)
                    indexAdd = waypointChecks.indexOf(waypoint);

            if(runTests)
                if(indexAdd<0){}
            //TODO log error

            Translation2d newWaypoint = null;

            b: {
                for (int i = trajectoryApproximation.indexOf(collisionStart)+2; i < trajectoryApproximation.size(); i++){
                    if (!obstacleMap.getCurrentMap().checkPixelHasObjectOrOffMap(trajectoryApproximation.get(i))){
                        int[] perpPixel = obstacleMap.getCurrentMap().getPerpPixel(startMinRadius, collisionStart, trajectoryApproximation.get(i));
                        newWaypoint = new Translation2d(perpPixel[0]/Constants.pixelsInMeter, perpPixel[1]/Constants.pixelsInMeter);
                        changedWaypoints.add(indexAdd, newWaypoint);
                        break b;
                    }
                }
                //TODO log error not found end of collision
            }

            if(newWaypoint == null) {
                startMinRadius += 1;
                maxLoops -= currentLoop - 1;
                currentLoop = 1;
            } else
            //programmed a waypoint removal based on a set radius from constants to stop these goddamn malformed spline exceptions
                for(int i = 0; i < changedWaypoints.size(); i++){
                    if(i != indexAdd && changedWaypoints.get(i).getDistance(newWaypoint) < Constants.waypointRemovalRadius) {
                        changedWaypoints.remove(i);
                        i--;
                        indexAdd--;
                    }
                }
        }

        obstacleMap.getCurrentMap().writeToFile("obstacle_map.map_data");

        return generateTrajectory(
                startPose,
                changedWaypoints,
                endPose,
                config
        );
    }

    public static Trajectory calculateTrajectory(final List<Pose2d> waypoints, ChassisSpeeds initial, boolean updateField, boolean runTests) {
        //Configures trajectory constraints
        TrajectoryConfig config = new TrajectoryConfig(kPathFollowingMaxVelMeters, kPathFollowingMaxAccelMeters);
        config.setStartVelocity(initial.vxMetersPerSecond);
        config.setEndVelocity(0);

        return calculateTrajectory(waypoints, config, updateField, runTests);
    }

    public static Trajectory calculateTrajectory(final List<Pose2d> waypoints, TrajectoryConfig config, boolean updateField, boolean runTests) {
        List<Translation2d> newWaypoints = new ArrayList<>();
        waypoints.forEach(a -> newWaypoints.add(a.getTranslation()));

        newWaypoints.remove(newWaypoints.size()-1);
        newWaypoints.remove(0);

        return calculateTrajectory(newWaypoints, config, waypoints.get(waypoints.size()-1), waypoints.get(0), updateField, runTests);
    }

    public static Trajectory calculateTrajectory(final List<Pose2d> waypoints, boolean changeToTranslation, TrajectoryConfig config, boolean updateField, boolean runTests) {
        Trajectory traj = new Trajectory();
        if(changeToTranslation){
            List<Translation2d> translations = new ArrayList<>();
            waypoints.forEach(a -> translations.add(a.getTranslation()));
            translations.remove(0);
            translations.remove(translations.size()-1);
            traj = calculateTrajectory(translations, config, waypoints.get(0), waypoints.get(waypoints.size()-1), updateField, runTests);
        }
        else
            for(int i = 0; i < waypoints.size()-1; i++)
                traj = traj.concatenate(calculateTrajectory(null, config, waypoints.get(i), waypoints.get(i+1), updateField, runTests));

        return traj;
    }
}