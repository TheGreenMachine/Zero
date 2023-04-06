package com.team1816.season.states;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Objects;

import static com.team1816.lib.subsystems.Subsystem.factory;
import static com.team1816.lib.subsystems.Subsystem.robotState;

/**
 * Main superstructure-style class and logical operator for handling and delegating subsystem tasks. Consists of an integrated
 * drivetrain with other subsystems and utilizes closed loop state dependent control via {@link RobotState}.
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
    private static Collector collector;
    private static Elevator elevator;


    /**
     * State
     */
    private final double maxAllowablePoseError = factory.getConstant(
        "maxAllowablePoseError",
        4
    );
    private final double minAllowablePoseError = factory.getConstant(
        "minAllowablePoseError",
        0.05
    );

    /**
     * Instantiates an Orchestrator with all its subsystems
     *
     * @param df  Drive.Factory (derives drivetrain)
     * @param led LedManager
     * @param el  Elevator
     * @param col Collector
     */
    @Inject
    public Orchestrator(Drive.Factory df, LedManager led, Collector col, Elevator el) {
        drive = df.getInstance();
        ledManager = led;
        collector = col;
        elevator = el;
    }

    /**
     * Actions
     */

    /** Superseded Odometry Handling */

    /**
     * Calculates the absolute pose of the drivetrain based on a single target
     *
     * @param target VisionPoint
     * @return Pose2d
     * @see VisionPoint
     */
    public Pose2d calculateSingleTargetTranslation(VisionPoint target) {
        Pose2d targetPos = FieldConfig.fieldTargets2023.get(target.id).toPose2d();

        double X = target.getX(), Y = target.getY();

        Translation2d cameraToTarget = new Translation2d(X, Y).rotateBy(robotState.getLatestFieldToCamera());
        Translation2d robotToTarget = cameraToTarget.plus(
            Constants.kCameraMountingOffset.getTranslation().rotateBy(
                robotState.fieldToVehicle.getRotation()
            )
        );
        Translation2d targetToRobot = robotToTarget.unaryMinus();

        Translation2d targetTranslation = targetToRobot.rotateBy(targetPos.getRotation());
        Pose2d p = targetPos.plus(
            new Transform2d(
                targetTranslation,
                targetPos.getRotation().rotateBy(Rotation2d.fromDegrees(180))
            )
        ); // inverse axis angle

        GreenLogger.log("Updated Pose: " + p);
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
        var cameraPoint = robotState.visibleTarget;
        if (!Objects.equals(cameraPoint, new VisionPoint()) && cameraPoint.id > 0) {
            var p = calculateSingleTargetTranslation(cameraPoint);
            Pose2d pose = new Pose2d(
                p.getX(),
                p.getY(),
                robotState.fieldToVehicle.getRotation()
            );
            ledManager.indicateStatus(LedManager.RobotStatus.ZEROING, LedManager.ControlState.BLINK);
            robotState.isPoseUpdated = true;
            return pose;
        }
//        List<Pose2d> poses = new ArrayList<>();
//        double sX = 0, sY = 0;
//        for (VisionPoint point : cameraPoint) {
//            var p = calculateSingleTargetTranslation(point);
//            sX += p.getX();
//            sY += p.getY();
//            poses.add(p);
//        }
//        if (cameraPoint.size() > 0) {
//            Pose2d pose = new Pose2d(
//                sX / cameraPoint.size(),
//                sY / cameraPoint.size(),
//                robotState.fieldToVehicle.getRotation()
//            );
//            robotState.isPoseUpdated = true;
//            return pose;
//        }
        GreenLogger.log("vision point bad - returning fieldToVehicle");
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
            GreenLogger.log(newRobotPose + " = new robot pose");
            drive.resetOdometry(newRobotPose);
            robotState.fieldToVehicle = newRobotPose;
            robotState.isPoseUpdated = true;
        }
    }

    public enum CONTROL_MODE {
        ALEPH_0,
        ALEPH_1
    }
}
