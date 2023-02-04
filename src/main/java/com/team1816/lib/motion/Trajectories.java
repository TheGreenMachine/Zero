package com.team1816.lib.motion;
import com.team1816.season.auto.paths.*;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.auto.paths.*;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.io.*;
import java.util.List;

public class Trajectories {
    public static void main(String[] args) {
        calcAllTrajectories(new LivingRoomPath());
        calcAllTrajectories(new DriveStraightPath());
    };
    public static void calcAllTrajectories(AutoPath path) {
        String name = formatClassName(path.getClass().getName());
        calcTrajectory(name, path.getWaypoints());
        calcTrajectory(name + "-reversed", path.getReflectedWaypoints());
        calcTrajectory(name + "-rotated", path.getRotatedWaypoints());
    };
    public static String formatClassName(String name) {
        String[] parts = name.split("\\.");
        return parts[parts.length - 1];
    }
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
}
