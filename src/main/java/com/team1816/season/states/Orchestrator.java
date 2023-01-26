package com.team1816.season.states;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.lib.subsystems.LedManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
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
     * @param led LedManager
     */
    @Inject
    public Orchestrator(Drive.Factory df, LedManager led) {
        drive = df.getInstance();
        ledManager = led;
        superstructureState = STATE.FAT_BOY;
    }

    /** Actions */

    /** Update Subsystem States */

    /** Superseded Odometry Handling */

    /**
     * Returns true if the pose of the drivetrain needs to be updated in a cached boolean system
     *
     * @return boolean
     */
    public boolean needsVisionUpdate() {
        if (!robotState.isPoseUpdated) {
            return true;
        }
        if (RobotBase.isSimulation() || RobotBase.isReal()) return false;
        boolean needsVisionUpdate =
            (
                Math.abs(
                    robotState.getCalculatedAccel().vxMetersPerSecond -
                        robotState.triAxialAcceleration[0]
                ) >
                    Constants.kMaxAccelDiffThreshold ||
                    Math.abs(
                        robotState.getCalculatedAccel().vyMetersPerSecond -
                            robotState.triAxialAcceleration[1]
                    ) >
                        Constants.kMaxAccelDiffThreshold ||
                    Math.abs(-9.8d - robotState.triAxialAcceleration[2]) >
                        Constants.kMaxAccelDiffThreshold
            );
        if (needsVisionUpdate) {
            robotState.isPoseUpdated = false;
        }
        return needsVisionUpdate; // placeHolder
    }

    /**
     * Calculates the absolute pose of the drivetrain based on a single target
     *
     * @param target VisionPoint
     * @return Pose2d
     * @see VisionPoint
     */
    public Pose2d calculateSingleTargetTranslation(VisionPoint target) {
        Pose2d targetPos = new Pose2d(
            FieldConfig.fieldTargets.get(target.id).getX(),
            FieldConfig.fieldTargets.get(target.id).getY(),
            new Rotation2d()
        );
        double X = target.getX(), Y = target.getY();
        Pose2d p = targetPos.plus(
            new Transform2d(
                new Translation2d(X, Y),
                robotState.getLatestFieldToCamera().rotateBy(Rotation2d.fromDegrees(180))
            )
        ); // inverse axis angle
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
            FieldConfig.fieldTargets.get(target.getFiducialId()).getX(),
            FieldConfig.fieldTargets.get(target.getFiducialId()).getY(),
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
        if (
            Math.abs(
                Math.hypot(
                    robotState.fieldToVehicle.getX() - newRobotPose.getX(),
                    robotState.fieldToVehicle.getY() - newRobotPose.getY()
                )
            ) > minAllowablePoseError
        ) {
            System.out.println(newRobotPose + " = new robot pose");
            drive.resetOdometry(newRobotPose);
            robotState.fieldToVehicle = newRobotPose;
            robotState.isPoseUpdated = true;
        }
    }

    /**
     * Base enum for Orchestrator states
     */
    public enum STATE {
        FAT_BOY,
        LITTLE_MAN,
    }
}
