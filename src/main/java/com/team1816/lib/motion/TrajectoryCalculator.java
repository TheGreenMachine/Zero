package com.team1816.lib.motion;
import com.team1816.season.auto.paths.*;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.auto.paths.*;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import org.apache.commons.io.FileUtils;

import java.io.*;
import java.util.List;

/**
 * Class that efficiently pre-calculates trajectories to limit resource utilization.
 * Generated Trajectories are located in src/resources/trajectories
 *
 * @see Trajectory
 */
public class TrajectoryCalculator {

    /**
     * Root directory that trajectories are placed in
     */
    private static final File directory = new File(System.getProperty("user.dir") + "/src/main/resources/trajectories/");

    /**
     * List of all AutoPaths that are to be calculated
     * @see AutoPath
     */
    private static final List<AutoPath> paths = List.of(
        new DriveStraightPath(),
        new LivingRoomPath()
    );

    /**
     * Entry point of gradle task that calculates trajectories
     * @param args
     */
    public static void main(String[] args) {
        deleteTrajectories();
        System.out.println("Calculating " + paths.size() + " Trajectories:");
        for (AutoPath path: paths) {
            calcAllTrajectoryFormats(path);
            System.out.println("\tCalculated " + path.getClass().getName());
        }
    }

    /**
     * Calculates all formats of the trajectories associated with an AutoPath {standard, reflected, rotated}
     * @param path AutoPath
     */
    public static void calcAllTrajectoryFormats(AutoPath path) {
        if (!directory.exists()) {
            directory.mkdir();
        }
        String name = formatClassName(path.getClass().getName());

        calcTrajectory(name, path.getWaypoints());
        calcTrajectory(name + "_Reflected", path.getReflectedWaypoints());
        calcTrajectory(name + "_Rotated", path.getRotatedWaypoints());

        if (path.getWaypointHeadings() != null) {
            calcHeadings(name + "Headings", path.getWaypoints(), path.getWaypointHeadings());
            calcHeadings(name + "Headings_Reflected", path.getReflectedWaypoints(), path.getReflectedWaypointHeadings());
            calcHeadings(name + "Headings_Rotated", path.getRotatedWaypoints(), path.getRotatedWaypointHeadings());
        }
    }

    /**
     * Formats the class name in regex by matching characters
     * @param name class name
     * @return name
     */
    public static String formatClassName(String name) {
        String[] parts = name.split("\\.");
        return parts[parts.length - 1];
    }

    /**
     * Calculates the Trajectory using the waypoints provided in the AutoPath and PathUtil
     * @param name path name
     * @param waypoints path waypoints
     * @see AutoPath
     * @see PathUtil
     */
    public static void calcTrajectory(String name, List<Pose2d> waypoints) {
        if (waypoints == null) {
            return;
        }
        var trajectory = PathUtil.generateTrajectory(name, true, waypoints, true);
        ObjectMapper mapper = new ObjectMapper();
        try {
            File file = new File(System.getProperty("user.dir") + "/src/main/resources/trajectories/" + name + ".json");
            if (!file.exists()) {
                file.createNewFile();
            }
            mapper.writeValue(file, trajectory.getStates());
        } catch (Exception e) {
            System.out.println("Error while writing JSON: " + e.getMessage());
        }
    }

    /**
     * Calculates the Headings associated with the AutoPath
     * @param name path name
     * @param waypoints path waypoints
     * @param headings path headings
     */
    public static void calcHeadings(String name, List<Pose2d> waypoints, List<Rotation2d> headings) {
        if (headings == null) {
            return;
        }

        var trajectoryHeadings = PathUtil.generateHeadings(name, true, waypoints, headings);
        ObjectMapper mapper = new ObjectMapper();
        try {
            File file = new File(System.getProperty("user.dir") + "/src/main/resources/trajectories/" + name + ".json");
            if (!file.exists()) {
                file.createNewFile();
            }
            mapper.writeValue(file, trajectoryHeadings);
        } catch (Exception e) {
            System.out.println("Error while writing JSON: " + e.getMessage());
        }
    }

    /**
     * Loads the Trajectory states into the associated JSON storing it
     * @param name path name
     * @return Trajectory
     * @see Trajectory
     */
    public static Trajectory loadTrajectory(String name) {
        ObjectMapper mapper = new ObjectMapper();
        try {
            List<Trajectory.State> list = mapper.readValue(
                new File(System.getProperty("user.dir") + "/src/main/resources/trajectories/" + name + ".json"),
                new TypeReference<List<Trajectory.State>>() { }
            );
            return new Trajectory(list);
        } catch (Exception e) {
            System.out.println("Error parsing path JSON: " + name + "\n" + e.getMessage());
            return new Trajectory();
        }
    }

    /**
     * Clears the Trajectories located in the resources directory
     */
    public static void deleteTrajectories() {
        try {
            FileUtils.cleanDirectory(new File(System.getProperty("user.dir") + "/src/main/resources/trajectories/"));
        } catch (Exception e) {
            System.out.println("Error deleting contents of trajectories directory" + e.getStackTrace());
        }
    }
}
