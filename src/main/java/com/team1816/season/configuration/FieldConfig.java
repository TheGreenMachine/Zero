package com.team1816.season.configuration;

import com.google.inject.Singleton;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@Singleton
public class FieldConfig {

    public static Field2d field;
    public static final HashMap<Integer, Pose3d> fieldTargets2023 = new HashMap<>() {
        {
            /**
             * April Tag Targets
             */

            /* Red Alliance Side */
            put(1, new Pose3d(new Translation3d(15.43, 1.132, 0.470), new Rotation3d())); // Red Edge Grid
            put(2, new Pose3d(new Translation3d(15.43, 4.482, 0.470), new Rotation3d())); // Red Co-Op Grid
            put(3, new Pose3d(new Translation3d(15.43, 2.808, 0.470), new Rotation3d())); // Red Feeder Edge Grid
            put(4, new Pose3d(new Translation3d(16.09, 6.782, 0.700), new Rotation3d())); // Blue Feeder Station

            /* Blue Alliance Side */
            put(5, new Pose3d(new Translation3d(0.310, 6.782, 0.700), new Rotation3d())); // Red Feeder Station
            put(6, new Pose3d(new Translation3d(0.970, 4.482, 0.470), new Rotation3d())); // Blue Feeder Edge Grid
            put(7, new Pose3d(new Translation3d(0.970, 2.808, 0.470), new Rotation3d())); // Blue Co-Op Grid
            put(8, new Pose3d(new Translation3d(0.970, 1.132, 0.470), new Rotation3d())); // Blue Edge Grid

            /* Test */
            put(51, new Pose3d(new Translation3d(), new Rotation3d()));
        }
    };

    public static void setupField(Field2d field) {
        if (FieldConfig.field != null) {
            // if field is already set up, do not set up again
            return;
        }
        FieldConfig.field = field;

        SmartDashboard.putData("Field", field);

        if (RobotBase.isSimulation()) {
            // set up april tags
            List<Pose2d> aprilTagPoses = new ArrayList<>();
            for (int i = 0; i <= 8; i++) {
                if (fieldTargets2023.get(i) == null) {
                    aprilTagPoses.add(
                        i,
                        new Pose2d(new Translation2d(-1, -1), new Rotation2d())
                    );
                    continue;
                }
                aprilTagPoses.add(
                    i, // if we want ids to be marked on each pose, we'll prob need to adjust the Field2DObject class (make our own?)
                    new Pose2d(
                        fieldTargets2023.get(i).getX(),
                        fieldTargets2023.get(i).getY(),
                        fieldTargets2023.get(i).getRotation().toRotation2d()
                    )
                );
            }
            field.getObject("April Tags").setPoses(aprilTagPoses);
        }
    }
}
