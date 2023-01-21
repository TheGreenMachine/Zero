package com.team1816.example;

import com.team1816.lib.auto.paths.AutoPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

public class ExampleAutoPath extends AutoPath {

    /**
     * To build your path, clone our cheesy-path repository (git clone https://github.com/TheGreenMachine/cheesy-path.git)
     * and follow instructions on the ReadME.
     * <p>
     * Here's a more in-depth guide if you want/need it->
     * https://docs.google.com/document/d/1jCRidlvIZ5hZg-OFc1bZWmaZgIgztRfeLw-Fo9ySd6s/edit?usp=sharing
     * </p>
     *
     * @return A list of points on the field that the robot must pass through - these points are linked together
     * into a trajectory for use in AutoModes
     */
    @Override
    protected List<Pose2d> getWaypoints() {
        // path looks like a deformed and mirrored C shape
        return List.of(
            new Pose2d(1.72, 0.49, Rotation2d.fromDegrees(0)),
            new Pose2d(6.76, 1.15, Rotation2d.fromDegrees(0)),
            new Pose2d(9.09, 1.46, Rotation2d.fromDegrees(45)),
            new Pose2d(9.04, 4.4, Rotation2d.fromDegrees(135)),
            new Pose2d(1.77, 4.93, Rotation2d.fromDegrees(180))
        );
    }

    /**
     * Note: this is only used for SwerveDrive robots b/c TankDrive heading is the same as its Pose2d's Rotation2d component.
     * Each waypoint should have a corresponding waypointHeading - more or less than the # of waypoints will cause faulty behavior
     *
     * @return a list of headings to go through when generating the full trajectory - as of now, we don't have anything
     * in chezy path that includes this, you just have to eyeball where the robot should face using the simulator.
     */
    @Override
    protected List<Rotation2d> getWaypointHeadings() {
        // this will make a SwerveDrive robot face the opponent alliance wall throughout the entire path
        return List.of(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0)
        );
    }

    @Override
    protected List<Pose2d> getReflectedWaypoints() {
        return null;
    }

    @Override
    protected List<Rotation2d> getReflectedWaypointHeadings() {
        return null;
    }

    /**
     * Honestly we shouldn't even have this here anymore
     *
     * @return just make this return true - we're always using the app
     */
    @Override
    protected boolean usingApp() {
        return true;
    } // TODO remove me :)
}
