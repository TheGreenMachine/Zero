package com.team1816.season.states;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
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
    private STATE orhcestratorState;
    private SCORE_LEVEL_STATE desiredScoreLevelState;
    private ELEMENT fieldElement;
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

        fieldElement = ELEMENT.NULL;
    }

    public void setOrchestratorState(STATE state) {
        orhcestratorState = state;
        robotState.orchestratorState = state; //TODO
    }

    public void setDesiredScoreLevelState(SCORE_LEVEL_STATE dsls) {
        desiredScoreLevelState = dsls;
        robotState.scoreLevelState = dsls; //TODO
    }

    /** Actions */

    /**
     * Sets the orchestrator to collect
     * @param collecting
     */
    public void setCollecting(boolean collecting) {
        if (collecting) {
            setOrchestratorState(STATE.COLLECT);
        }
        setCollectorCollecting(collecting, fieldElement == ELEMENT.CUBE);
        // setElevatorCollecting(collecting);
    }

    /**
     * Sets the orchestrator to collect
     * @param collecting collecting
     * @param cube cube
     */
    public void setCollecting(boolean collecting, boolean cube) {
        setCollectorCollecting(collecting, cube);
        // setElevatorCollecting(collecting);
    }

    /**
     * Sets the orchestrator to score
     * @param scoring scoring
     */
    public void setScoring(boolean scoring) {
        setCollectorScoring(scoring);
//        if (desiredScoreLevelState == SCORE_LEVEL_STATE.MIN) {
//            setElevatorScoring(scoring, Elevator.EXTENSION_STATE.MIN);
//        } else if (desiredScoreLevelState == SCORE_LEVEL_STATE.MID) {
//            setElevatorScoring(scoring, Elevator.EXTENSION_STATE.MID);
//        } else {
//            setElevatorScoring(scoring, Elevator.EXTENSION_STATE.MAX);
//        }
    }

    /**
     * Sets the desired state of the collector to collect
     * @param collecting collecting
     * @param cube field element
     */
    public void setCollectorCollecting(boolean collecting, boolean cube) {
        if (collecting) {
            if (cube) {
                fieldElement = ELEMENT.CUBE;
                collector.setDesiredState(ControlMode.PercentOutput, Collector.PIVOT_STATE.UP, Collector.ROLLER_STATE.INTAKE);
            } else {
                fieldElement = ELEMENT.CONE;
                collector.setDesiredState(ControlMode.Velocity, Collector.PIVOT_STATE.DOWN, Collector.ROLLER_STATE.INTAKE);
            }
        } else {
            collector.setDesiredState(ControlMode.Velocity, Collector.PIVOT_STATE.UP, Collector.ROLLER_STATE.STOP);
        }
    }

    /**
     * Sets the desired state of the collector to score based on the stored field element
     * @param scoring scoring
     */
    public void setCollectorScoring(boolean scoring) {
        if (scoring) {
            if (fieldElement == ELEMENT.CONE) {
                collector.setDesiredState(ControlMode.Velocity, Collector.PIVOT_STATE.UP, Collector.ROLLER_STATE.OUTTAKE);
            } else {
                collector.setDesiredState(ControlMode.PercentOutput, Collector.PIVOT_STATE.UP, Collector.ROLLER_STATE.OUTTAKE);
            }
        } else {
            collector.setDesiredState(ControlMode.Velocity, Collector.PIVOT_STATE.UP, Collector.ROLLER_STATE.STOP);
        }
        fieldElement = ELEMENT.NULL;
    }

    /**
     * Sets the desired state of the elevator to collect
     * @param collecting collecting
     */
    public void setElevatorCollecting(boolean collecting) {
        if (collecting) {
            elevator.setDesiredState(Elevator.ANGLE_STATE.COLLECT, Elevator.EXTENSION_STATE.MIN);
        } else {
            elevator.setDesiredState(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN);
        }
    }

    /**
     * Sets the desired state of the elevator to score based on the desired level fed by robot state
     * @param scoring scoring
     * @param level desired level
     */
    public void setElevatorScoring(boolean scoring, Elevator.EXTENSION_STATE level) {
        if (scoring) {
            elevator.setDesiredState(Elevator.ANGLE_STATE.SCORE, level);
        } else {
            elevator.setDesiredState(Elevator.ANGLE_STATE.STOW, Elevator.EXTENSION_STATE.MIN);
        }
    }



    /** TODO: Update Subsystem States */
    public void update() {
    }

    /** Superseded Odometry Handling */

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
    public enum ELEMENT {
        NULL,
        CONE,
        CUBE
    }

    public enum STATE {
        COLLECT,
        SCORE,
        STOW
    }

    public enum SCORE_LEVEL_STATE {
        MIN,
        MID,
        MAX
    }

    public enum CONTROL_MODE {
        ALEPH_0,
        ALEPH_1
    }
}
