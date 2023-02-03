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
    public static void calculate() {
        calcAllTrajectories(new LivingRoomPath());
        calcAllTrajectories(new DriveStraightPath());
    };
    public static void calcAllTrajectories(AutoPath path) {
        String name = path.getClass().getName();
        calcTrajectory(name, path.getWaypoints());
        calcTrajectory(name + "-reversed", path.getReflectedWaypoints());
        calcTrajectory(name + "-rotated", path.getRotatedWaypoints());
    };
    public static void calcTrajectory(String name, List<Pose2d> waypoints) {
        if (waypoints == null) {
            return;
        }
        var trajectory = PathUtil.generateTrajectory(name, true, waypoints, true);
        ObjectMapper mapper = new ObjectMapper();
        try {
            mapper.writeValue(
                new File(System.getProperty("user.dir") + "/src/resources/trajectories/" + name + ".json"), 
                trajectory.getStates()
            );
        } catch (Exception e) {
            System.out.println("Error while writing JSON: " + e.getMessage());
        }
    }
    public static Trajectory loadTrajectory(String name) {
        ObjectMapper mapper = new ObjectMapper();
        try {
            List<Trajectory.State> list = mapper.readValue(
                Trajectories.class.getResource("trajectories/" + name + ".json"),
                new TypeReference<List<Trajectory.State>>() { }
            );
            return new Trajectory(list);
        } catch (Exception e) {
            System.out.println("Error parsing path JSON: " + name + "\n" + e.getMessage());
            return new Trajectory();
        }
    }
}
