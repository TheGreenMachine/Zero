package com.team1816.season.states;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.turret.Turret;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.season.subsystems.Collector;
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

    private static Collector collector;

    /**
     * Subsystems
     */
    private static Drive drive;
    private static LedManager ledManager;
    private static Camera camera;

    /**
     * State
     */
    private OBJECT desiredObject;
    private STATE desiredState;
    private boolean isCube;
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
     */
    @Inject
    public Orchestrator(Drive.Factory df, LedManager led) {
        drive = df.getInstance();
        ledManager = led;
        desiredObject = OBJECT.CONE;
        desiredState = STATE.STOP;
        collector = Injector.get(Collector.class);
        camera = Injector.get(Camera.class);
        isCube = false;
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

    public void setCollectCone(boolean pressed) {
        isCube = false;
        if (pressed) {
            collector.setDesiredState(isCube, Collector.PIVOT_STATE.DOWN, Collector.COLLECTOR_STATE.COLLECT);
        } else {
            collector.setDesiredState(isCube, Collector.PIVOT_STATE.DOWN, Collector.COLLECTOR_STATE.STOP);
        }
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

        public void setCollectCube ( boolean pressed){
            isCube = true;
            if (pressed) {
                collector.setDesiredState(isCube, Collector.PIVOT_STATE.UP, Collector.COLLECTOR_STATE.COLLECT);
            } else {
                collector.setDesiredState(isCube, Collector.PIVOT_STATE.UP, Collector.COLLECTOR_STATE.STOP);
            }
        }

        public void setEject(boolean pressed){
            if (pressed) {
                collector.setDesiredState(isCube, Collector.PIVOT_STATE.UP, Collector.COLLECTOR_STATE.EJECT);
            } else {
                collector.setDesiredState(isCube, Collector.PIVOT_STATE.UP, Collector.COLLECTOR_STATE.STOP);
            }
        }



        /**
         * Base enum for Orchestrator states
         */
        public enum OBJECT {
            CONE,
            CUBE
        }

        public enum STATE {
            COLLECTING,
            EJECTING,
            STOP
        }
    }
