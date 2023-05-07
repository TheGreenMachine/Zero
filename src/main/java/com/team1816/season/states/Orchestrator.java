package com.team1816.season.states;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.vision.Camera;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.auto.commands.AlignElevatorCommand;
import com.team1816.season.auto.commands.AutoScoreCommand;
import com.team1816.season.auto.commands.TargetAlignCommand;
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
    private static Camera camera;
    private static LedManager ledManager;
    private static Collector collector;
    private static Elevator elevator;

    /**
     * Properties
     */
    private Thread alignElevatorThread;
    private Thread autoScoreThread;
    private Thread autoTargetAlignThread;

    public static boolean runningAutoTarget = false;
    public static boolean runningAutoTargetAlign = false;
    public static boolean runningAutoAlign = false;
    public static boolean runningAutoScore = false;

    /**
     * Instantiates an Orchestrator with all its subsystems
     *
     * @param df  Drive.Factory (derives drivetrain)
     * @param led LedManager
     * @param el  Elevator
     * @param col Collector
     */
    @Inject
    public Orchestrator(Drive.Factory df, Camera cam, LedManager led, Collector col, Elevator el) {
        drive = df.getInstance();
        camera = cam;
        ledManager = led;
        collector = col;
        elevator = el;
    }

    /**
     * Actions
     */

    /**
     * Uses AutoCommand framework and A* path finding algorithms to drive and align to a specific
     *
     * @param level - Extension State
     * @see com.team1816.lib.auto.commands.AutoCommand
     * @see TargetAlignCommand
     * @see com.team1816.lib.auto.PathFinder
     */
    public void autoTargetAlign(Elevator.EXTENSION_STATE level) {
        if (!runningAutoTargetAlign) {
            runningAutoTargetAlign = true;
            updatePoseWithCamera();
            double distance = robotState.fieldToVehicle.getTranslation().getDistance(robotState.target.getTranslation());
            if (distance < Constants.kMinTrajectoryDistance) {
                GreenLogger.log("Distance to target is " + distance + " m");
                GreenLogger.log("Too close to target! can not start trajectory! setting elevator extension to: " + level.name());
                AlignElevatorCommand command = new AlignElevatorCommand(level);
                autoTargetAlignThread = new Thread(command::run);
            } else {
                GreenLogger.log("Drive trajectory action started!");
                TargetAlignCommand command = new TargetAlignCommand(level);
                autoTargetAlignThread = new Thread(command::run);
            }
            ledManager.indicateStatus(LedManager.RobotStatus.AUTONOMOUS, LedManager.ControlState.SOLID);
            autoTargetAlignThread.start();
        } else {
            autoTargetAlignThread.stop();
            GreenLogger.log("Stopped! driving to trajectory canceled!");
            ledManager.indicateStatus(LedManager.RobotStatus.ENABLED, LedManager.ControlState.SOLID);
            runningAutoTargetAlign = !runningAutoTargetAlign;
        }
    }

    /**
     * Aligns to score at a low node
     *
     * @see com.team1816.lib.auto.commands.AutoCommand
     * @see AlignElevatorCommand
     */
    public void alignMin() {
        if (!runningAutoAlign) {
            runningAutoAlign = true;
            GreenLogger.log("Auto align action started!");
            AlignElevatorCommand command = new AlignElevatorCommand(Elevator.EXTENSION_STATE.MIN);
            alignElevatorThread = new Thread(command::run);
            alignElevatorThread.start();
        } else {
            alignElevatorThread.stop();
            GreenLogger.log("Stopped! Auto align cancelled!");
            runningAutoAlign = !runningAutoAlign;
        }
    }

    /**
     * Aligns to score at a middle node
     *
     * @see com.team1816.lib.auto.commands.AutoCommand
     * @see AlignElevatorCommand
     */
    public void alignMid() {
        if (!runningAutoAlign) {
            runningAutoAlign = true;
            GreenLogger.log("Auto align action started!");
            AlignElevatorCommand command = new AlignElevatorCommand(Elevator.EXTENSION_STATE.MID);
            alignElevatorThread = new Thread(command::run);
            alignElevatorThread.start();
        } else {
            alignElevatorThread.stop();
            GreenLogger.log("Stopped! Auto align cancelled!");
            runningAutoAlign = !runningAutoAlign;
        }
    }

    /**
     * Aligns to score at a high node
     *
     * @see com.team1816.lib.auto.commands.AutoCommand
     * @see AlignElevatorCommand
     */
    public void alignMax() {
        if (!runningAutoAlign) {
            runningAutoAlign = true;
            GreenLogger.log("Auto align action started!");
            AlignElevatorCommand command = new AlignElevatorCommand(Elevator.EXTENSION_STATE.MAX);
            alignElevatorThread = new Thread(command::run);
            alignElevatorThread.start();
        } else {
            alignElevatorThread.stop();
            GreenLogger.log("Stopped! Auto align cancelled!");
            runningAutoAlign = !runningAutoAlign;
        }
    }

    /**
     * Scores the current game piece
     *
     * @see com.team1816.lib.auto.commands.AutoCommand
     * @see AutoScoreCommand
     */
    public void autoScore() {
        if (!runningAutoScore) {
            runningAutoScore = true;
            GreenLogger.log("Auto Score action started!");
            AutoScoreCommand command = new AutoScoreCommand(collector.getCurrentGameElement(), elevator.getDesiredExtensionState());
            autoScoreThread = new Thread(command::run);
            autoScoreThread.start();
        } else {
            autoScoreThread.stop();
            GreenLogger.log("Stopped! Auto Score sequence cancelled!");
            runningAutoScore = !runningAutoScore;
        }
    }

    /**
     * Clears executable threads
     */
    public void clearThreads() {
        if (autoScoreThread != null && autoScoreThread.isAlive()) {
            autoScoreThread.stop();
        }
        if (alignElevatorThread != null && alignElevatorThread.isAlive()) {
            alignElevatorThread.stop();
        }
        if (autoTargetAlignThread != null && autoTargetAlignThread.isAlive()) {
            autoTargetAlignThread.stop();
        }
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
        if (FieldConfig.fiducialTargets.containsKey(target.id)) {
            Pose2d targetPos = FieldConfig.fiducialTargets.get(target.id).toPose2d();
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
        } else {
            return robotState.fieldToVehicle;
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
            FieldConfig.fiducialTargets.get(target.getFiducialId()).getX(),
            FieldConfig.fiducialTargets.get(target.getFiducialId()).getY(),
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
        if (camera.isUsingMultiTargetOdometryCalculation()) {
            // Multi-target pose estimation
            var cameraPoints = robotState.visibleTargets;
            double sX = 0, sY = 0, count = 0;
            for (VisionPoint point : cameraPoints) {
                if (!Objects.equals(point, new VisionPoint()) && point.id >= 0) {
                    var p = calculateSingleTargetTranslation(point);
                    sX += p.getX();
                    sY += p.getY();
                    count++;
                }
            }
            if (count > 0) {
                Pose2d pose = new Pose2d(
                    sX / count,
                    sY / count,
                    robotState.fieldToVehicle.getRotation()
                );
                robotState.isPoseUpdated = true;
                return pose;
            }
        } else {
            // Single target pose estimation
            var cameraPoint = robotState.superlativeTarget;
            if (!Objects.equals(cameraPoint, new VisionPoint()) && cameraPoint.id >= 0) {
                var p = calculateSingleTargetTranslation(cameraPoint);
                Pose2d pose = new Pose2d(
                    p.getX(),
                    p.getY(),
                    robotState.fieldToVehicle.getRotation()
                );
                robotState.isPoseUpdated = true;
                return pose;
            }
        }
        GreenLogger.log("Vision Points bad - returning fieldToVehicle");
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
            ) > drive.minAllowablePoseError
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
