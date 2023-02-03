package com.team1816.lib.auto.paths;

import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.Symmetry;
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
     * State: contains information about whether the trajectory should be rotated and how
     */
    boolean rotated;

    public AutoPath() {
    }

    public AutoPath(Color color) {
        if (Constants.fieldSymmetry == Symmetry.AXIS && color == Color.RED) {
            setReflected(true);
            setRotated(false);
        } else if (Constants.fieldSymmetry == Symmetry.ORIGIN && color == Color.RED) {
            setReflected(false);
            setRotated(true);
        } else {
            setReflected(false);
            setRotated(false);
        }
    }

    /**
     * Sets the reflected state to passed argument
     *
     * @param reflected - reflected
     */
    protected void setReflected(boolean reflected) {
        this.reflected = reflected;
    }

    /**
     * Sets the rotated state to passed argument
     *
     * @param rotated - rotated
     */
    protected void setRotated(boolean rotated) {
        this.rotated = rotated;
    }

    /**
     * Returns a list of Pose2d's that define the trajectory /path to be followed
     *
     * @return waypoints
     */
    public abstract List<Pose2d> getWaypoints();

    /**
     * Returns a list of Pose2d's that define the reflected trajectory/path to be followed
     *
     * @return waypoints
     */
    public List<Pose2d> getReflectedWaypoints() {
        List<Pose2d> waypoints = getWaypoints();
        List<Pose2d> reflectedWaypoints = new ArrayList<>();
        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d waypoint = new Pose2d(2 * Constants.fieldCenterX - waypoints.get(i).getX(), waypoints.get(i).getY(), Rotation2d.fromDegrees(180 - waypoints.get(i).getRotation().getDegrees()));
            reflectedWaypoints.add(i, waypoint);
        }
        return reflectedWaypoints;
    }

    /**
     * Returns a list of Pose2d's that define the rotated trajectory/path to be followed
     *
     * @return waypoints
     */
    public List<Pose2d> getRotatedWaypoints() {
        List<Pose2d> waypoints = getWaypoints();
        List<Pose2d> rotatedWaypoints = new ArrayList<>();
        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d waypoint = new Pose2d(2 * Constants.fieldCenterX - waypoints.get(i).getX(), 2 * Constants.fieldCenterY - waypoints.get(i).getY(), Rotation2d.fromDegrees(180 + waypoints.get(i).getRotation().getDegrees()));
            rotatedWaypoints.add(i, waypoint);
        }
        return rotatedWaypoints;
    }

    /**
     * Returns a list of Rotation2d's corresponding to the rotation respect to a trajectory
     *
     * @return waypointHeadings
     */
    protected abstract List<Rotation2d> getWaypointHeadings();

    /**
     * Returns a list of Rotation2d's corresponding to the rotation respect to a reflected trajectory
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
     * Returns a list of Rotation2d's corresponding to the rotation respect to a rotated trajectory
     *
     * @return waypointHeadings
     */
    protected List<Rotation2d> getRotatedWaypointHeadings() {
        List<Rotation2d> waypointHeadings = getWaypointHeadings();
        List<Rotation2d> rotatedWaypointHeadings = new ArrayList<>();
        for (int i = 0; i < waypointHeadings.size(); i++) {
            var waypointRotation = Rotation2d.fromDegrees(180 + waypointHeadings.get(i).getDegrees());
            rotatedWaypointHeadings.add(i, waypointRotation);
        }
        return rotatedWaypointHeadings;
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
            if (!reflected && !rotated) {
                trajectory = PathUtil.generateTrajectory(getClass().getName(), usingApp(), getWaypoints());
            } else if (reflected) {
                trajectory = PathUtil.generateTrajectory(getClass().getName() + "-reversed", usingApp(), getReflectedWaypoints());
            } else {
                trajectory = PathUtil.generateTrajectory(getClass().getName() + "-rotated", usingApp(), getRotatedWaypoints());
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
            String name = getClass().getName();
            if (!reflected && !rotated) {
                headings =
                    PathUtil.generateHeadings(
                        name,
                        usingApp(),
                        getWaypoints(),
                        getWaypointHeadings()
                    );
            } else if (reflected) {
                headings =
                    PathUtil.generateHeadings(
                        name + "-reversed",
                        usingApp(),
                        getReflectedWaypoints(),
                        getReflectedWaypointHeadings()
                    );
            } else {
                headings =
                    PathUtil.generateHeadings(
                        name + "-rotated",
                        usingApp(),
                        getRotatedWaypoints(),
                        getRotatedWaypointHeadings()
                    );
            }
        }
        return headings;
    }
}
