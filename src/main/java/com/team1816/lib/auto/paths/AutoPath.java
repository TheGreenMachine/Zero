package com.team1816.lib.auto.paths;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
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
     * State: contains information about whether the trajectory should be reflected and how
     */
    boolean reflected;

    /**
     * Sets the reflected state to passed argument
     *
     * @param reflected - reflected
     */
    protected void setReflected(boolean reflected) {
        this.reflected = reflected;
    }

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
     * Returns a list of Pose2d's that define the reflected trajectory/path to be followed
     *
     * @return waypoints
     */
    protected List<Pose2d> getReflectedWaypoints() {
        List<Pose2d> waypoints = getWaypoints();
        List<Pose2d> reflectedWaypoints = new ArrayList<>();
        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d waypoint = new Pose2d(2 * Constants.fieldCenterX - waypoints.get(i).getX(), waypoints.get(i).getY(), Rotation2d.fromDegrees(180 - waypoints.get(i).getRotation().getDegrees()));
            reflectedWaypoints.add(i, waypoint);
        }
        return reflectedWaypoints;
    }

    /**
     * Returns a list of Rotation2d's corresponding to the rotation respect to the reflected trajectory
     *
     * @return waypointHeadings
     */
    protected List<Rotation2d> getReflectedWaypointHeadings() {
        List<Rotation2d> waypointHeadings = getWaypointHeadings();
        List<Rotation2d> reflectedWaypointHeadings = new ArrayList<>();
        for (int i = 0; i < waypointHeadings.size(); i++) {
            var waypointRotation = Rotation2d.fromDegrees(180 - waypointHeadings.get(i).getDegrees());
            reflectedWaypointHeadings.add(i, waypointRotation);
        }
        return reflectedWaypointHeadings;
    }

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
            if (!reflected) {
                trajectory = PathUtil.generateTrajectory(usingApp(), getWaypoints());
            } else {
                trajectory = PathUtil.generateTrajectory(usingApp(), getReflectedWaypoints());
            }
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
            if (!reflected) {
                headings =
                    PathUtil.generateHeadings(
                        usingApp(),
                        getWaypoints(),
                        getWaypointHeadings()
                    );
            } else {
                headings =
                    PathUtil.generateHeadings(
                        usingApp(),
                        getReflectedWaypoints(),
                        getReflectedWaypointHeadings()
                    );
            }
        }
        return headings;
    }
}
