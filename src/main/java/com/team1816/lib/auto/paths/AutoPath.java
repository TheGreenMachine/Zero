package com.team1816.lib.auto.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.List;

/**
 * Abstract class containing all information necessary for running a trajectory as a TrajectoryAction
 *
 * @see com.team1816.lib.auto.actions.TrajectoryAction
 * @see Trajectory
 */
public abstract class AutoPath {

    /**
     * State: contains information about the trajectory that defines the path
     */
    Trajectory trajectory;
    /**
     * State: headings corresponding to the path that can be transposed onto the trajectory for omni-directional drivetrains like swerve
     */
    List<Rotation2d> headings;

    /**
     * Returns a list of Pose2d's that define the trajectory /path to be followed
     *
     * @return waypoints
     */
    protected abstract List<Pose2d> getWaypoints();

    /**
     * Returns a list of Rotation2d's corresponding to the rotation respect to the trajectory
     *
     * @return waypointHeadings
     */
    protected abstract List<Rotation2d> getWaypointHeadings();

    /**
     * Checks if the path was made using the CheesyPath web application.
     * If false, the starting pose of the trajectory will be set to the default starting pose.
     *
     * @return boolean usingApp
     */
    protected abstract boolean usingApp();

    /**
     * Returns the generated trajectory associated with the AutoPath
     *
     * @return trajectory
     * @see Trajectory
     */
    public Trajectory getAsTrajectory() {
        if (trajectory == null) {
            trajectory = PathUtil.generateTrajectory(usingApp(), getWaypoints());
        }
        return trajectory;
    }

    /**
     * Returns the list of trajectory headings that are transposed onto the path, ignored for tank based drivetrains
     *
     * @return trajectoryHeadings
     * @see com.team1816.lib.auto.actions.TrajectoryAction
     */
    public List<Rotation2d> getAsTrajectoryHeadings() {
        if (headings == null) {
            headings =
                PathUtil.generateHeadings(
                    usingApp(),
                    getWaypoints(),
                    getWaypointHeadings()
                );
        }
        return headings;
    }
}
