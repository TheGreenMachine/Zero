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
    public static final HashMap<Integer, Pose3d> fieldTargets2023 = new HashMap<>() {
        {
            /**
             * April Tag Targets
             */

            /* Red Alliance Side */
            put(1, new Pose3d(new Translation3d(15.453, 1.282, 0.470), new Rotation3d(0, 0, Math.PI))); // Red Edge Grid
            put(2, new Pose3d(new Translation3d(15.453, 2.959, 0.470), new Rotation3d(0, 0, Math.PI))); // Red Co-Op Grid
            put(3, new Pose3d(new Translation3d(15.453, 4.635, 0.470), new Rotation3d(0, 0, Math.PI))); // Red Feeder Edge Grid
            put(4, new Pose3d(new Translation3d(16.106, 7.126, 0.700), new Rotation3d(0, 0, Math.PI))); // Blue Feeder Station

            /* Blue Alliance Side */
            put(5, new Pose3d(new Translation3d(0.354, 7.126, 0.700), new Rotation3d(0, 0, 0))); // Red Feeder Station
            put(6, new Pose3d(new Translation3d(1.007, 4.635, 0.470), new Rotation3d(0, 0, 0))); // Blue Feeder Edge Grid
            put(7, new Pose3d(new Translation3d(1.007, 2.959, 0.470), new Rotation3d(0, 0, 0))); // Blue Co-Op Grid
            put(8, new Pose3d(new Translation3d(1.007, 1.282, 0.470), new Rotation3d(0, 0, 0))); // Blue Edge Grid

            /* Test */
            put(51, new Pose3d(new Translation3d(Constants.fieldCenterX, Constants.fieldCenterY, 0.1), new Rotation3d()));
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
