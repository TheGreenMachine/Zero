package com.team1816.season.states;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.turret.Turret;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.subsystems.LedManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

import static com.team1816.lib.subsystems.Subsystem.factory;
import static com.team1816.lib.subsystems.Subsystem.robotState;

/**
 * Main superstructure-style class and logical operator for handling and delegating subsystem tasks. Consists of an integrated
 * drivetrain with other subsystems and utilizes closed loop state dependent control via RobotState.
 *
 * @see RobotState
 */
@Singleton
public class Orchestrator {

    /**
     * Subsystems
     */
    private static Drive drive;
    private static LedManager ledManager;


    private static Turret turret;
    /**
     * State
     */
    private STATE superstructureState;
    private final double maxAllowablePoseError = factory.getConstant(
        "maxAllowablePoseError",
        4
    );
    private final double minAllowablePoseError = factory.getConstant(
        "minAllowablePoseError",
        0.1
    );

    /**
     * Instantiates an Orchestrator with all its subsystems
     *
     * @param df  Drive.Factory (derives drivetrain)
     * @param tur Turret
     * @param led LedManager
     */
    @Inject
    public Orchestrator(Drive.Factory df, Turret tur, LedManager led) {
        drive = df.getInstance();
        turret = tur;
        ledManager = led;
        superstructureState = STATE.FAT_BOY;
    }

    /** TODO: Actions */

    /** TODO: Update Subsystem States */

    /**
     * Calculates the absolute pose of the drivetrain based on a single target
     *
     * @param target VisionPoint
     * @return Pose2d
     * @see VisionPoint
     */
    public Pose2d calculateSingleTargetTranslation(VisionPoint target) {
        Pose2d targetPos = new Pose2d(
            FieldConfig.fieldTargets2023.get(target.id).getX(),
            FieldConfig.fieldTargets2023.get(target.id).getY(),
            new Rotation2d()
        );
        System.out.println("Target " + target.id + " pose: " + targetPos);
        double X = target.getX(), Y = target.getY();
        Pose2d p = targetPos.plus(
            new Transform2d(
                new Translation2d(X, Y),
                robotState.getLatestFieldToCamera().rotateBy(Rotation2d.fromDegrees(180))
            )
        ); // inverse axis angle
        System.out.println("Updated Pose: " + p);
        return p;
    }

    /**
     * Calculates the absolute pose of the drivetrain based on a single target using PhotonVision's library
     *
     * @param target VisionPoint
     * @return Pose2d
     * @see org.photonvision.targeting.PhotonTrackedTarget
     */
    public Pose2d photonCalculateSingleTargetTranslation(PhotonTrackedTarget target) {
        Pose2d targetPos = new Pose2d(
            FieldConfig.fieldTargets2023.get(target.getFiducialId()).getX(),
            FieldConfig.fieldTargets2023.get(target.getFiducialId()).getY(),
            new Rotation2d()
        );
        Translation2d targetTranslation = target.getBestCameraToTarget().getTranslation().toTranslation2d();
        Transform2d targetTransform = new Transform2d(targetTranslation, robotState.getLatestFieldToCamera());
        return PhotonUtils.estimateFieldToCamera(targetTransform, targetPos);
    }


    /**
     * Calculates the absolute pose of the drivetrain as a function of all visible targets
     *
     * @return Pose2d
     */
    public Pose2d calculatePoseFromCamera() {
        var cameraPoints = robotState.visibleTargets;
        System.out.println("Targets size: " + robotState.visibleTargets.size());
        List<Pose2d> poses = new ArrayList<>();
        double sX = 0, sY = 0;
        for (VisionPoint point : cameraPoints) {
            var p = calculateSingleTargetTranslation(point);
            sX += p.getX();
            sY += p.getY();
            poses.add(p);
        }
        if (cameraPoints.size() > 0) {
            Pose2d pose = new Pose2d(
                sX / cameraPoints.size(),
                sY / cameraPoints.size(),
                robotState.fieldToVehicle.getRotation()
            );
            robotState.isPoseUpdated = true;
            return pose;
        }
        return robotState.fieldToVehicle;
    }

    /**
     * Updates the pose of the drivetrain based on specified criteria
     */
    public void updatePoseWithCamera() {
        Pose2d newRobotPose = calculatePoseFromCamera();
        System.out.println(newRobotPose + " = new robot pose");
        drive.resetOdometry(newRobotPose);
        robotState.fieldToVehicle = newRobotPose;
        robotState.isPoseUpdated = true;
    }

    /**
     * Base enum for Orchestrator states
     */
    public enum STATE {
        FAT_BOY,
        LITTLE_MAN,
    }
}
