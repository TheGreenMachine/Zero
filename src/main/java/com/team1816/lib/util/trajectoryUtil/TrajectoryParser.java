package com.team1816.lib.util.trajectoryUtil;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility that parses trajectories calculated with the TrajectoryCalculator
 * @see TrajectoryCalculator
 */
public class TrajectoryParser {

    /**
     * Root directory that trajectories are placed in
     */
    private static final File directory = new File(System.getProperty("user.dir") + "/src/main/resources/trajectories/");

    /**
     * Static object mapper for parsing
     */
    private static final ObjectMapper mapper = new ObjectMapper();

    /**
     * Parses trajectory
     *
     * @param name auto path name
     * @return trajectory
     */
    public static Trajectory parseTrajectory(String name) {
        File file = new File(directory.getPath() + name + ".json");
        try {
            List<Trajectory.State> trajectoryStates = mapper.readValue(
                file, new TypeReference<List<Trajectory.State>>() {}
            );
            var trajectory = new Trajectory(trajectoryStates);
            return trajectory;
        } catch (Exception e) {
            System.out.println("Error parsing trajectory: " + e.getStackTrace());
        }
        return new Trajectory();
    }

    /**
     * Parses reflected trajectory
     *
     * @param name auto path name
     * @return trajectory
     */
    public static Trajectory parseReflectedTrajectory(String name) {
        return parseTrajectory(name+"_Reflected");
    }

    /**
     * Parses rotated trajectory
     *
     * @param name auto path name
     * @return trajectory
     */
    public static Trajectory parseRotatedTrajectory(String name) {
        return parseTrajectory(name+"_Rotated");
    }

    /**
     * Parses trajectory headings
     * NOTE: need to append "Headings" to {name}
     *
     * @param name auto path name
     * @return trajectory headings
     */
    public static List<Rotation2d> parseTrajectoryHeadings(String name) {
        File file = new File(directory.getPath() + name + ".json");
        try {
            List<Rotation2d> trajectoryHeadings = mapper.readValue(
                file, new TypeReference<List<Rotation2d>>() {}
            );
            return trajectoryHeadings;
        } catch (Exception e) {
            System.out.println("Error parsing trajectory: " + e.getStackTrace());
        }
        return new ArrayList<>();
    }

    /**
     * Parses reflected trajectory headings
     * NOTE: need to append "Headings" to {name}
     *
     * @param name auto path name
     * @return trajectory headings
     */
    public static List<Rotation2d> parseReflectedTrajectoryHeadings(String name) {
        return parseTrajectoryHeadings(name+"_Reflected");
    }

    /**
     * Parses rotated trajectory headings
     * NOTE: need to append "Headings" to {name}
     *
     * @param name auto path name
     * @return trajectory headings
     */
    public static List<Rotation2d> parseRotatedTrajectoryHeadings(String name) {
        return parseTrajectoryHeadings(name+"_Rotated");
    }
}
