package com.team1816.lib.subsystems.vision;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.visionUtil.GreenSimVisionSystem;
import com.team1816.lib.util.visionUtil.GreenSimVisionTarget;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.states.RobotState;
import com.team1816.lib.subsystems.LedManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

/**
 * Camera interface that utilizes PhotonVision for target detection and measurement
 */
@Singleton
public class Camera extends Subsystem {

    /**
     * Components
     */
    static LedManager led;
    private PhotonCamera cam;
    private GreenSimVisionSystem simVisionSystem;

    /**
     * Constants
     */
    private static final String NAME = "camera";
    private final double CAMERA_HEIGHT_METERS = 0.7493;

    /**
     * State
     */
    public boolean cameraEnabled;
    private PhotonTrackedTarget bestTrackedTarget;


    /**
     * Instantiates a camera with the base subsystem properties
     * @param ledManager LedManager
     * @param inf Infrastructure
     * @param rs RobotState
     */
    @Inject
    public Camera(LedManager ledManager, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        led = ledManager;
        cameraEnabled = this.isImplemented();

        if (RobotBase.isSimulation()) {
            simVisionSystem =
                new GreenSimVisionSystem(
                    "ZED", // TODO make this not zed
                    90,
                    60,
                    Constants.kCameraMountingAngleY,
                    Constants.EmptyTransform2d,
                    CAMERA_HEIGHT_METERS,
                    9000,
                    3840,
                    1080,
                    0
                );
            for (int i = 0; i <= 8; i++) {
                if (FieldConfig.fieldTargets2023.get(i) == null) {
                    continue;
                }
                simVisionSystem.addSimVisionTarget(
                    new GreenSimVisionTarget(
                        new Pose2d(
                            FieldConfig.fieldTargets2023.get(i).getX(),
                            FieldConfig.fieldTargets2023.get(i).getY(),
                            FieldConfig.fieldTargets2023.get(i).getRotation().toRotation2d()
                        ),
                        FieldConfig.fieldTargets2023.get(i).getZ(),
                        .1651,
                        .1651,
                        i
                    )
                );
            }
        }
        PhotonCamera.setVersionCheckEnabled(false);
        if (cameraEnabled) {
            cam = new PhotonCamera("IMX219");
        }
    }

    /**
     * Sets the camera to be enabled
     * @param cameraEnabled boolean
     */
    public void setCameraEnabled(boolean cameraEnabled) {
        if (this.isImplemented()) {
            this.cameraEnabled = cameraEnabled;
            led.setCameraLed(cameraEnabled);
        } else {
            System.out.println("Camera Not Implemented...");
        }
    }



    /**
     * Periodically reads inputs and polls visible camera targets
     */
    public void readFromHardware() {
        if (RobotBase.isSimulation()) {
            simVisionSystem.moveCamera(
                new Transform2d(
                    robotState.getFieldToTurretPos(),
                    robotState.fieldToVehicle
                ), // sim vision inverts this Transform when calculating robotPose
                CAMERA_HEIGHT_METERS,
                Constants.kCameraMountingAngleY
            );
            simVisionSystem.processFrame(robotState.fieldToVehicle);
            robotState.field
                .getObject("camera")
                .setPose(
                    robotState.fieldToVehicle.transformBy(
                        new Transform2d(
                            robotState.fieldToVehicle,
                            robotState.getFieldToTurretPos()
                        )
                    )
                );
        }
        robotState.visibleTargets = getPoints();
    }


    /**
     * Polls targets from the camera and returns the best target as a list of VisionPoints (reduces computational overhead)
     * @return List of VisionPoint
     *
     * @see VisionPoint
     */
    public ArrayList<VisionPoint> getPoints() {
        ArrayList<VisionPoint> targets = new ArrayList<>();
        if (isImplemented()) {
            VisionPoint p = new VisionPoint();
            var result = cam.getLatestResult();
            if (!result.hasTargets()) {
                return targets;
            }
            var bestTarget = result.getBestTarget();
            p.id = bestTarget.getFiducialId();
            p.cameraToTarget = bestTarget.getBestCameraToTarget(); // missing method in PhotonTrackedTarget
            targets.add(p);
        } else {
            System.out.println("camera not returning points b/c camera not implemented");
        }
        return targets;
    }

    /**
     * Polls targets from the camera and returns all targets as a list of VisionPoints
     *
     * @return List of VisionPoints
     */
    public ArrayList<VisionPoint> getPointsAlternate() {
        ArrayList<VisionPoint> targets = new ArrayList<>();
        var result = cam.getLatestResult();
        if (!result.hasTargets()) {
            return targets;
        }

        double m = 0xFFFFFF; // big number
        var principal_RANSAC = new PhotonTrackedTarget();

        for (PhotonTrackedTarget target : result.targets) {
            var p = new VisionPoint();
            if (target.getBestCameraToTarget() != null) {
                p.cameraToTarget = target.getBestCameraToTarget();
                p.id = target.getFiducialId();
                targets.add(p);

                if (m > p.cameraToTarget.getTranslation().getNorm()) {
                    m = p.cameraToTarget.getTranslation().getNorm();
                    principal_RANSAC = target;
                }
            }
        }

        bestTrackedTarget = principal_RANSAC;

        return targets;
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
                FieldConfig.fieldTargets2023.get(target.id).getX(),
                FieldConfig.fieldTargets2023.get(target.id).getY(),
                new Rotation2d()
        );
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
     * Functionality: nonexistent
     */
    @Override
    public void writeToHardware() {}

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {}

    /**
     * Tests the camera
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        if (isImplemented()) {
            led.setCameraLed(true);
            Timer.delay(2);
            led.setCameraLed(false);
        }
        return true;
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {
    }

}
