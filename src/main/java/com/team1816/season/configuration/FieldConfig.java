package com.team1816.season.configuration;

import com.google.inject.Singleton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
             * April Tag Targets
             */
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
