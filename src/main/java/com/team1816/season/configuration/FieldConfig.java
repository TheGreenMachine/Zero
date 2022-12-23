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
    public static final HashMap<Integer, Pose3d> fieldTargets = new HashMap<>() {
        {
            /**
             * Retro-reflective Tape Targets
             */
            put(-1, new Pose3d(new Translation3d(8.230, 4.115, 2.434), new Rotation3d()));

            /**
             * April Tag Targets
             */

            /** blue alliance side */
            put(0, new Pose3d(new Translation3d(0.000, 7.510, 0.886), new Rotation3d())); // Blue Hangar Panel

            put(1, new Pose3d(new Translation3d(3.320, 5.588, 1.725), new Rotation3d())); // upper hangar target - Blue Hangar Truss - Hub

            put(2, new Pose3d(new Translation3d(3.072, 5.249, 1.376), new Rotation3d())); // target - Blue Hangar Truss - Side

            put(3, new Pose3d(new Translation3d(0.008, 5.059, 0.806), new Rotation3d())); // Blue Station 2 Wall

            put(4, new Pose3d(new Translation3d(0.008, 3.575, 0.806), new Rotation3d())); // Blue Station 3 Wall

            /** blue human player station */
            put(5, new Pose3d(new Translation3d(0.125, 1.656, 0.891), new Rotation3d())); // Blue Terminal Near Station

            put(6, new Pose3d(new Translation3d(0.877, 0.879, 0.891), new Rotation3d())); // Blue Mid Terminal

            put(7, new Pose3d(new Translation3d(1.619, 0.095, 0.891), new Rotation3d())); // Blue End Terminal

            /** red alliance side */
            put(
                10,
                new Pose3d(new Translation3d(16.460, 0.589, 0.886), new Rotation3d())
            ); // Red Hangar Panels

            put(
                11,
                new Pose3d(new Translation3d(13.240, 2.750, 1.725), new Rotation3d())
            ); // upper hangar target

            put(
                12,
                new Pose3d(new Translation3d(13.395, 2.838, 1.376), new Rotation3d())
            ); // lower hangar target

            put(
                13,
                new Pose3d(new Translation3d(16.459, 3.114, 0.806), new Rotation3d())
            ); // red Station 2 Wall

            put(
                14,
                new Pose3d(new Translation3d(16.459, 4.655, 0.806), new Rotation3d())
            ); // red Station 3 Wall

            // red human player station
            put(
                15,
                new Pose3d(new Translation3d(16.339, 6.453, 0.894), new Rotation3d())
            );

            put(
                16,
                new Pose3d(new Translation3d(15.594, 7.231, 0.891), new Rotation3d())
            );

            put(
                17,
                new Pose3d(new Translation3d(14.851, 8.007, 0.891), new Rotation3d())
            );

            // lower hub targets
            put(40, new Pose3d(new Translation3d(7.878, 4.851, 0.703), new Rotation3d()));

            put(41, new Pose3d(new Translation3d(7.435, 3.697, 0.703), new Rotation3d()));

            put(42, new Pose3d(new Translation3d(8.589, 3.254, 0.703), new Rotation3d()));

            put(43, new Pose3d(new Translation3d(9.032, 4.408, 0.703), new Rotation3d()));

            // upper hub targets
            put(50, new Pose3d(new Translation3d(7.684, 4.330, 2.408), new Rotation3d()));

            put(51, new Pose3d(new Translation3d(8.020, 3.576, 2.408), new Rotation3d()));

            put(52, new Pose3d(new Translation3d(8.775, 3.912, 2.408), new Rotation3d()));

            put(53, new Pose3d(new Translation3d(8.439, 4.667, 2.408), new Rotation3d()));
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
            for (int i = 0; i <= 53; i++) {
                if (fieldTargets.get(i) == null) {
                    aprilTagPoses.add(
                        i,
                        new Pose2d(new Translation2d(-1, -1), new Rotation2d())
                    );
                    continue;
                }
                aprilTagPoses.add(
                    i, // if we want ids to be marked on each pose, we'll prob need to adjust the Field2DObject class (make our own?)
                    new Pose2d(
                        fieldTargets.get(i).getX(),
                        fieldTargets.get(i).getY(),
                        fieldTargets.get(i).getRotation().toRotation2d()
                    )
                );
            }
            field.getObject("April Tags").setPoses(aprilTagPoses);
        }
    }
}
