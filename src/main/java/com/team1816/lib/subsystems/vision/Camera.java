package com.team1816.lib.subsystems.vision;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.visionUtil.GreenSimVisionSystem;
import com.team1816.lib.util.visionUtil.GreenSimVisionTarget;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;

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
    private final double CAMERA_HEIGHT_METERS = 0.15;

    /**
     * State
     */
    public boolean cameraEnabled;
    private PhotonTrackedTarget bestTrackedTarget;
    private DoubleArrayLogEntry visionTargetLogger;


    /**
     * Instantiates a camera with the base subsystem properties
     *
     * @param ledManager LedManager
     * @param inf        Infrastructure
     * @param rs         RobotState
     */
    @Inject
    public Camera(LedManager ledManager, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        led = ledManager;
        cameraEnabled = this.isImplemented();

        if (RobotBase.isSimulation()) {
            simVisionSystem =
                new GreenSimVisionSystem(
                    "ZED",
                    90,
                    60,
                    Constants.kCameraMountingOffset.getRotation().getDegrees(),
                    new Transform2d(
                        Constants.kCameraMountingOffset.getTranslation(),
                        Constants.EmptyRotation2d
                    ),
                    CAMERA_HEIGHT_METERS,
                    25,
                    3840,
                    1080,
                    0
                );
            for (int i = 0; i <= 8; i++) {
                if (FieldConfig.fiducialTargets.get(i) == null) {
                    continue;
                }
                simVisionSystem.addSimVisionTarget(
                    new GreenSimVisionTarget(
                        new Pose2d(
                            FieldConfig.fiducialTargets.get(i).getX(),
                            FieldConfig.fiducialTargets.get(i).getY(),
                            FieldConfig.fiducialTargets.get(i).getRotation().toRotation2d()
                        ),
                        FieldConfig.fiducialTargets.get(i).getZ(),
                        .1651,
                        .1651,
                        i
                    )
                );
            }
        }
        PhotonCamera.setVersionCheckEnabled(false);
        if (cameraEnabled) {
            cam = new PhotonCamera("snakeyes");
            if (Constants.kLoggingRobot) {
                visionTargetLogger = new DoubleArrayLogEntry(DataLogManager.getLog(), "Camera/SeenPoints");
            }
        }
    }

    /**
     * Sets the camera to be enabled
     *
     * @param cameraEnabled boolean
     */
    public void setCameraEnabled(boolean cameraEnabled) {
        if (this.isImplemented()) {
            this.cameraEnabled = cameraEnabled;
        } else {
            GreenLogger.log("Camera Not Implemented...");
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
                            Constants.kCameraMountingOffset.getTranslation(),
                            Constants.EmptyRotation2d
                        )
                    )
                );
        }
        robotState.superlativeTarget = getPoint();
        robotState.visibleTargets = getPoints();

        if (Constants.kLoggingRobot && cameraEnabled) {
            var targetPose = robotState.fieldToVehicle;

            Pose3d aprilTagPose = FieldConfig.fiducialTargets.get(robotState.superlativeTarget.id);
            if (aprilTagPose != null) {
                targetPose = aprilTagPose.toPose2d();
            }

            visionTargetLogger.append(new double[]{targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
        }
    }


    /**
     * Polls targets from the camera and returns the best target as a singular VisionPoint
     *
     * @return VisionPoint
     * @see VisionPoint
     */
    public VisionPoint getPoint() {
        VisionPoint targets = new VisionPoint();
        if (isImplemented()) {
            VisionPoint p = new VisionPoint();
            var result = cam.getLatestResult();
            if (!result.hasTargets()) {
                return targets;
            }
            var bestTarget = result.getBestTarget();
            p.id = bestTarget.getFiducialId();
            p.cameraToTarget = bestTarget.getBestCameraToTarget(); // missing method in PhotonTrackedTarget
            return p;
        }
        return targets;
    }

    /**
     * Polls targets from the camera and returns the best target as a list of VisionPoints (reduces computational overhead)
     *
     * @return List of VisionPoint
     * @see VisionPoint
     */
    public ArrayList<VisionPoint> getPoints() {
        ArrayList<VisionPoint> targets = new ArrayList<>();
        if (isImplemented()) {
            var result = cam.getLatestResult();
            if (!result.hasTargets()) {
                return targets;
            }
            var bestTarget = result.getBestTarget();
            for (PhotonTrackedTarget target : result.getTargets()) {
                VisionPoint p = new VisionPoint();
                p.id = target.getFiducialId();
                p.cameraToTarget = target.getBestCameraToTarget();
                targets.add(p);
            }
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
     * Functionality: nonexistent
     */
    @Override
    public void writeToHardware() {
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void zeroSensors() {
    }

    /**
     * Tests the camera
     *
     * @return true if tests passed
     */
    @Override
    public boolean testSubsystem() {
        return true;
    }

    /**
     * Functionality: nonexistent
     */
    @Override
    public void stop() {
    }

}
