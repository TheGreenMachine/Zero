package com.team1816.lib.subsystems.vision;

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
import com.team1816.season.subsystems.LedManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;

@Singleton
public class Camera extends Subsystem {

    // Components
    static LedManager led;
    private PhotonCamera cam;
    private GreenSimVisionSystem simVisionSystem;

    // Constants
    private static final String NAME = "camera";
    public final boolean camImplemented;
    public final boolean usingCamToUpdatePose;

    private static final double CAMERA_FOCAL_LENGTH = 700; // px
    private static final double VIDEO_WIDTH = 1280; // px
    private static final double VIDEO_HEIGHT = 720; // px
    private static final double CAMERA_DFOV = 100; // degrees
    private static final double CAMERA_HFOV = 85;
    private final double CAMERA_HEIGHT_METERS = 0.7493; // meters
    private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(
        Constants.kTargetHeight
    );
    private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(
        Constants.kCameraMountingAngleY
    );

    private final double MAX_DIST = factory.getConstant(NAME, "maxDist", 20);

    // state
    private PhotonTrackedTarget bestTrackedTarget;

    @Inject
    public Camera(LedManager ledManager, Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        led = ledManager;
        camImplemented = this.isImplemented();
        usingCamToUpdatePose = factory.getConstant(NAME,"usingCamToUpdatePose") > 0;

        // if using sim, set up fake vision system
        if (RobotBase.isSimulation()) {
            // defining sim vision system
            simVisionSystem =
                new GreenSimVisionSystem(
                    "ZED-M",
                    90,
                    60,
                    Constants.kCameraMountingAngleY,
                    new Transform2d(
                        new Translation2d(-.12065, .13335),
                        Constants.EmptyRotation2d
                    ), //TODO update this value
                    CAMERA_HEIGHT_METERS,
                    9000,
                    3840,
                    1080,
                    0
                );
            // adding april tags to our field for our simCamera to "see"
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
                        .1651, // Estimated width & height of the AprilTag
                        .1651,
                        i
                    )
                );
            }
        }

        PhotonCamera.setVersionCheckEnabled(false);
        // only actually make a cam if cam implemented
        // prevents "no coprocessor found" error when sim is booted up w/ out a camera
        if(camImplemented){
            cam = new PhotonCamera("snakeyes");
        }
    }

    @Override
    public boolean testSubsystem() {
        if (warnIfCameraOff()) {
            led.setCameraLed(true);
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
            led.setCameraLed(false);
        }
        return true;
    }

    /**
     * stuff normally done in read to hardware performed by coprocessor (PI4).
     * We only need to use read from hardware if we're faking out the subsystem in sim
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
    }

    /**
     * Get distance to goal
     * @return distance ALONG THE GROUND to the target - NOT the hypotenuse
     */
    public double getDistance() {
        if(warnIfCameraOff()){
            if(usingCamToUpdatePose){
                return robotState.getDistanceToGoal();
            } else {
                var result = cam.getLatestResult();
                if (!result.hasTargets()) {
                    return -1;
                }

                // note that this is coded for 3D mode on PhotonVision (b/c we're grabbing a Transform3d)
                var bestTargetTransform = result.getBestTarget().getBestCameraToTarget();
                return Math.hypot(bestTargetTransform.getX(), bestTargetTransform.getY());
            }
        }
        return -1;
    }

    public double getDeltaX() {
        if(warnIfCameraOff()) {
            if (RobotBase.isSimulation()) { // simulated feedback loop
                return simulateDeltaX();
            }
            var result = cam.getLatestResult();
            if (!result.hasTargets()) {
                return -1;
            }

            return result.getBestTarget().getYaw();
        }
        return -1;
    }

    /**
     * stuff normally done in write to hardware performed by coprocessor (PI4). No need to do anything on our end
     */
    @Override
    public void writeToHardware() {}

    @Override
    public void stop() {}

    @Override
    public void zeroSensors() {}

    /**
     * We can only this if vision becomes solid enough that we can just keep in on throughout the match.
     * Rn we are not using this
     * @return list of all targets (apriltags or reflective tape) seen by camera
     */
    public ArrayList<VisionPoint> getPoints() {
        ArrayList<VisionPoint> targets = new ArrayList<>();
        if(this.isImplemented()){
            VisionPoint p = new VisionPoint();
            var result = cam.getLatestResult();
            if (!result.hasTargets()) {
                return targets;
            }
            var bestTarget = result.getBestTarget();
            p.id = bestTarget.getFiducialId();
//        p.cameraToTarget = bestTarget.getCameraToTarget(); // missing method in PhotonTrackedTarget
            targets.add(p);
        } else {
            System.out.println("camera not returning points b/c camera not implemented");
        }

        return targets;
    }

    /**
     * This may be @Deprecated. TODO double check if we actually need to use this or if the simCam deals with this for us
     * @return deltaX based on methemetiquess
     */
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

    public boolean warnIfCameraOff() {
        if(!camImplemented){
            System.out.println("cam not implemented - not performing action");
        }
        return camImplemented;
    }
}
