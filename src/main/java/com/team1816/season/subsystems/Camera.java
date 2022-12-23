package com.team1816.season.subsystems;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.lib.util.visionUtil.GreenPhotonCamera;
import com.team1816.lib.util.visionUtil.GreenSimVisionSystem;
import com.team1816.lib.util.visionUtil.GreenSimVisionTarget;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import org.photonvision.*;
import org.photonvision.targeting.*;

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
     * Properties
     */
    private static final String NAME = "camera";

    private static final double CAMERA_FOCAL_LENGTH = 700; // px
    private static final double VIDEO_WIDTH = 1280; // px
    private static final double VIDEO_HEIGHT = 720; // px

    private static final double CAMERA_DFOV = 100; // degrees

    private final double MAX_DIST = factory.getConstant(NAME, "maxDist", 20);
    private static final double CAMERA_HFOV = 85;

    private final double CAMERA_HEIGHT_METERS = 0.7493; // meters
    private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(
        Constants.kTargetHeight
    );
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(
        Constants.kCameraMountingAngleY
    );

    /**
     * State
     */
    private boolean cameraEnabled;
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

        if (RobotBase.isSimulation()) {
            simVisionSystem =
                new GreenSimVisionSystem(
                    "ZED-M",
                    90,
                    60,
                    Constants.kCameraMountingAngleY,
                    new Transform2d(
                        new Translation2d(-.12065, .13335),
                        Constants.EmptyRotation
                    ),
                    CAMERA_HEIGHT_METERS,
                    9000,
                    3840,
                    1080,
                    0
                );
            for (int i = 0; i <= 53; i++) {
                if (FieldConfig.fieldTargets.get(i) == null) {
                    continue;
                }
                simVisionSystem.addSimVisionTarget(
                    new GreenSimVisionTarget(
                        new Pose2d(
                            FieldConfig.fieldTargets.get(i).getX(),
                            FieldConfig.fieldTargets.get(i).getY(),
                            FieldConfig.fieldTargets.get(i).getRotation().toRotation2d()
                        ),
                        FieldConfig.fieldTargets.get(i).getZ(),
                        .1651, // Width of the AprilTag
                        .1651, // Height of the AprilTag
                        i
                    )
                );
            }
        }
        GreenPhotonCamera.setVersionCheckEnabled(false);
        cam = new PhotonCamera("microsoft"); // Camera name
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
     * Returns if the camera is enabled
     * @return cameraEnabled
     */
    public boolean isEnabled() {
        return cameraEnabled;
    }

    /**
     * Toggles the enabled state of the camera
     */
    public void toggleEnabled() {
        setCameraEnabled(!cameraEnabled);
    }

    /**
     * Stops the camera
     */
    public void stop() {}

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
     * Polls targets from the camera and returns the best target as a list of VisionPoints (reduces computational overhead)
     * @return List of VisionPoint
     * @see VisionPoint
     */
    public ArrayList<VisionPoint> getPoints() {
        ArrayList<VisionPoint> targets = new ArrayList<>();
        VisionPoint p = new VisionPoint();
        var result = cam.getLatestResult();
        if (!result.hasTargets()) {
            return targets;
        }
        var bestTarget = result.getBestTarget();
        p.id = bestTarget.getFiducialId();
        p.cameraToTarget = bestTarget.getCameraToTarget();
        targets.add(p);
        return targets;
    }

    /**
     * Polls targets from the camera and returns all targets as a list of VisionPoints
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
            if (target.getCameraToTarget() != null) {
                p.cameraToTarget = target.getCameraToTarget();
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
     * Returns the distance to the goal (direct reference to RobotState)
     * @return distance (meters)
     */
    @Deprecated
    public double getDistance() {
        return robotState.getDistanceToGoal();
    }

    /**
     * Returns the pixel / angular difference of the target to the center of the camera
     * @return deltaX
     */
    @Deprecated
    public double getDeltaX() {
        if (RobotBase.isSimulation()) { //simulate feedback loop
            return simulateDeltaX();
        }
        if (bestTrackedTarget == null) {
            getPoints();
        }
        return bestTrackedTarget.getYaw();
    }

    /**
     * Tests the camera
     * @return true if tests passed
     */
    public boolean testSubsystem() {
        if (this.isImplemented()) {
            setCameraEnabled(true);
            Timer.delay(2);
            if (getDistance() < 0 || getDistance() > MAX_DIST) {
                System.out.println("getDistance failed test!");
                return false;
            } else if (
                getDeltaX() < -CAMERA_HFOV / 2d || getDeltaX() > CAMERA_HFOV / 2d
            ) {
                System.out.println("getDeltaX failed test!");
                return false;
            }
            setCameraEnabled(false);
        }
        return true;
    }

    /**
     * Simulates a deltaX calculation (would just be yaw)
     * @return double
     */
    @Deprecated
    public double simulateDeltaX() {
        double opposite =
            Constants.fieldCenterY - robotState.getFieldToTurretPos().getY();
        double adjacent =
            Constants.fieldCenterX - robotState.getFieldToTurretPos().getX();
        double targetTurretAngle = Math.atan(opposite / adjacent);
        if (adjacent < 0) {
            targetTurretAngle += Math.PI;
        }
        targetTurretAngle *= 180 / Math.PI;
        double currentTurretAngle = robotState
            .getFieldToTurretPos()
            .getRotation()
            .getDegrees();
        if (currentTurretAngle < 0 && adjacent < 0) {
            currentTurretAngle += 360;
        }
        return ((currentTurretAngle - targetTurretAngle)); // scaling for the feedback loop
    }
}
