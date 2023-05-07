package com.team1816.season.configuration;

import com.google.inject.Singleton;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A class that stores the positions of location targets used for global localization with the help of a vision system
 */
@Singleton
public class FieldConfig {

    public static Field2d field;
    public static final HashMap<Integer, Pose3d> fiducialTargets = new HashMap<>() {
        {
            /**
             * April Tag Targets
             */

            /* Red Alliance Side */
            put(1, new Pose3d(new Translation3d(15.514, 1.072, 0.470), new Rotation3d(0, 0, Math.PI))); // Red Edge Grid
            put(2, new Pose3d(new Translation3d(15.514, 2.748, 0.470), new Rotation3d(0, 0, Math.PI))); // Red Co-Op Grid
            put(3, new Pose3d(new Translation3d(15.514, 4.424, 0.470), new Rotation3d(0, 0, Math.PI))); // Red Feeder Edge Grid
            put(4, new Pose3d(new Translation3d(16.178, 6.750, 0.700), new Rotation3d(0, 0, Math.PI))); // Blue Feeder Station

            /* Blue Alliance Side */
            put(5, new Pose3d(new Translation3d(0.361, 6.750, 0.700), new Rotation3d(0, 0, 0))); // Red Feeder Station
            put(6, new Pose3d(new Translation3d(1.027, 4.424, 0.470), new Rotation3d(0, 0, 0))); // Blue Feeder Edge Grid
            put(7, new Pose3d(new Translation3d(1.027, 2.748, 0.470), new Rotation3d(0, 0, 0))); // Blue Co-Op Grid
            put(8, new Pose3d(new Translation3d(1.027, 1.072, 0.470), new Rotation3d(0, 0, 0))); // Blue Edge Grid
        }
    };

    public static void setupField(Field2d field) {
        if (FieldConfig.field != null) {
            return;
        }
        FieldConfig.field = field;

        SmartDashboard.putData("Field", field);

        if (RobotBase.isSimulation()) {
            // Initialize April Tag Poses
            List<Pose2d> aprilTagPoses = new ArrayList<>();
            for (int i = 0; i <= 8; i++) {
                if (!fiducialTargets.containsKey(i)) {
                    aprilTagPoses.add(
                        i,
                        new Pose2d(new Translation2d(-1, -1), new Rotation2d())
                    );
                    continue;
                }
                aprilTagPoses.add(
                    i,
                    new Pose2d(
                        fiducialTargets.get(i).getX(),
                        fiducialTargets.get(i).getY(),
                        fiducialTargets.get(i).getRotation().toRotation2d()
                    )
                );
            }
            field.getObject("April Tags").setPoses(aprilTagPoses);
        }
    }
}
