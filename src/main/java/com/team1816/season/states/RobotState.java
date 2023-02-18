package com.team1816.season.states;

import com.google.inject.Singleton;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is responsible for logging the robot's actual states and estimated states.
 * Including superstructure and subsystem states.
 */

@Singleton
public class RobotState {

    /**
     * Odometry and field characterization
     */
    public final Field2d field = new Field2d();
    public Pose2d fieldToVehicle = Constants.EmptyPose2d;
    public Pose2d extrapolatedFieldToVehicle = Constants.EmptyPose2d;
    public Pose2d target = Constants.EmptyPose2d;
    public Rotation2d vehicleToTurret = Constants.EmptyRotation2d;
    public Pose2d fieldToTurret = Constants.EmptyPose2d;
    public ChassisSpeeds deltaVehicle = new ChassisSpeeds(); // velocities of vehicle
    public ChassisSpeeds calculatedVehicleAccel = new ChassisSpeeds(); // accel values calculated by watching drivetrain encoders
    public Double[] triAxialAcceleration = new Double[]{0d, 0d, 0d};
    public boolean isPoseUpdated = true;
    public double vehicleToFloorProximityCentimeters = 0;
    public double drivetrainTemp = 0;

    /**
     * Inertial characterization
     */
    public Pose3d fieldToCG = Constants.EmptyPose3d;
    public Rotation3d inertialOrientationState = Constants.EmptyRotation3d;
    public Quaternion inertialReferenceOrientationState = Constants.EmptyQuaternion; // utilizes active multiplication

    /**
     * Orchestrator states
     */
    public Orchestrator.STATE orchestratorState = Orchestrator.STATE.STOW;
    public Orchestrator.SCORE_LEVEL_STATE scoreLevelState = Orchestrator.SCORE_LEVEL_STATE.MIN;
    public Elevator.EXTENSION_STATE actualElevatorExtensionState = Elevator.EXTENSION_STATE.MIN;
    public Elevator.ANGLE_STATE actualElevatorAngleState = Elevator.ANGLE_STATE.STOW;
    public Collector.ROLLER_STATE actualCollectorRollerState = Collector.ROLLER_STATE.STOP;
    public Collector.PIVOT_STATE actualCollectorPivotState = Collector.PIVOT_STATE.UP;
    public List<VisionPoint> visibleTargets = new ArrayList<>();

    /**
     * Initializes RobotState and field
     */
    public RobotState() {
        resetPosition();
        FieldConfig.setupField(field);
    }

    /**
     * Resets drivetrain and turret position to a specified pose of drivetrain and rotation of turret
     *
     * @param initial_field_to_vehicle
     * @param initial_vehicle_to_turret
     */
    public synchronized void resetPosition(
        Pose2d initial_field_to_vehicle,
        Rotation2d initial_vehicle_to_turret
    ) {
        resetPosition(initial_field_to_vehicle);
        vehicleToTurret = initial_vehicle_to_turret;
    }

    /**
     * Resets drivetrain position to a specified pose of drivetrain
     *
     * @param initial_field_to_vehicle
     */
    public synchronized void resetPosition(Pose2d initial_field_to_vehicle) {
        fieldToVehicle = initial_field_to_vehicle;
    }

    /**
     * Resets the drivetrain to its default "zero" pose
     *
     * @see Constants
     */
    public synchronized void resetPosition() {
        resetPosition(Constants.kDefaultZeroingPose);
    }

    /**
     * Resets all values stored in RobotState
     */
    public synchronized void resetAllStates() {
        deltaVehicle = new ChassisSpeeds();
        calculatedVehicleAccel = new ChassisSpeeds();
        triAxialAcceleration = new Double[]{0d, 0d, 0d};
        orchestratorState = Orchestrator.STATE.STOW;
        scoreLevelState = Orchestrator.SCORE_LEVEL_STATE.MIN;
        actualElevatorAngleState = Elevator.ANGLE_STATE.STOW;
        actualElevatorExtensionState = Elevator.EXTENSION_STATE.MIN;
        actualCollectorRollerState = Collector.ROLLER_STATE.STOP;
        actualCollectorPivotState = Collector.PIVOT_STATE.UP;
        isPoseUpdated = true;
        visibleTargets.clear();
        drivetrainTemp = 0;
        vehicleToFloorProximityCentimeters = 0;
        target = Constants.fieldCenterPose;
    }

    /**
     * Returns rotation of the turret with respect to the field
     *
     * @return Rotation2d
     */
    public Rotation2d getLatestFieldToTurret() {
        return fieldToTurret.getRotation();
    }

    /**
     * Returns rotation of the camera with respect to the field
     *
     * @return Rotation2d
     * @see Orchestrator#calculateSingleTargetTranslation(VisionPoint) ()
     */
    public Rotation2d getLatestFieldToCamera() {
        return fieldToTurret.getRotation();
    }

    /**
     * Returns pose of the turret with respect ot the field
     *
     * @return Pose2d
     */
    public synchronized Pose2d getFieldToTurretPos() {
        return fieldToTurret;
    }

    /**
     * Returns the estimated pose of the turret with respect to the field based on a look-ahead time
     *
     * @return Pose2d
     */
    public synchronized Pose2d getEstimatedFieldToTurretPos() {
        return new Pose2d(
            extrapolatedFieldToVehicle
                .transformBy(
                    new Transform2d(
                        Constants.kTurretMountingOffset,
                        Constants.EmptyRotation2d
                    )
                )
                .getTranslation(),
            getLatestFieldToTurret()
        );
    }

    /**
     * Returns the estimated / calculated acceleration of the robot based on sensor readings
     *
     * @return ChassisSpeeds
     */
    public synchronized ChassisSpeeds getCalculatedAccel() {
        return calculatedVehicleAccel;
    }

    /**
     * Returns the distance from the goal based on the pose of the robot
     *
     * @return distance (meters)
     */
    public double getDistanceToGoal() {
        double estimatedDistanceToGoalMeters = fieldToVehicle
            .getTranslation()
            .getDistance(Constants.targetPos.getTranslation());
        return estimatedDistanceToGoalMeters;
    }

    /**
     * Outputs real-time telemetry data to Shuffleboard / SmartDashboard
     */
    public synchronized void outputToSmartDashboard() {
        field.setRobotPose(fieldToVehicle);
    }
}
