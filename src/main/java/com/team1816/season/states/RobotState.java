package com.team1816.season.states;

import com.google.inject.Singleton;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.PathFinder;
import com.team1816.lib.util.visionUtil.VisionPoint;
import com.team1816.season.configuration.Constants;
import com.team1816.season.configuration.FieldConfig;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
    public Color allianceColor = Color.BLUE;
    public Pose2d fieldToVehicle = Constants.EmptyPose2d;
    public Pose2d driverRelativeFieldToVehicle = Constants.EmptyPose2d;
    public Pose2d extrapolatedFieldToVehicle = Constants.EmptyPose2d;
    public Pose2d target = Constants.fieldCenterPose;
    public Rotation2d vehicleToTurret = Constants.EmptyRotation2d;
    public Pose2d fieldToTurret = Constants.EmptyPose2d;
    public ChassisSpeeds deltaVehicle = new ChassisSpeeds(); // velocities of vehicle
    public ChassisSpeeds calculatedVehicleAccel = new ChassisSpeeds(); // calculated acceleration of vehicle
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
    public Elevator.EXTENSION_STATE actualElevatorExtensionState = Elevator.EXTENSION_STATE.MIN;
    public Elevator.ANGLE_STATE actualElevatorAngleState = Elevator.ANGLE_STATE.STOW;

    public Collector.ROLLER_STATE actualCollectorRollerState = Collector.ROLLER_STATE.STOP;
    public Collector.PIVOT_STATE actualCollectorPivotState = Collector.PIVOT_STATE.STOW;
    public Collector.GAME_ELEMENT actualGameElement = Collector.GAME_ELEMENT.CONE;

    public double actualElevatorAngle = 0;
    public double actualElevatorExtensionInches = 0; // INCHES
    public double actualCollectorAngle = 0;
    public VisionPoint visibleTarget = new VisionPoint();

    public boolean gameElementChanged = false;

    public final Mechanism2d mechCanvas = new Mechanism2d(3, 3);
    public final MechanismRoot2d root = mechCanvas.getRoot("ElevatorArm", 1.3, 0.38);
    public final MechanismLigament2d simArm = root.append(new MechanismLigament2d("elevator", Elevator.kElevatorMinLength, 90));
    public final MechanismLigament2d simCollector = simArm.append(new MechanismLigament2d("collector", Collector.kCollectorLength, 90));


    /**
     * Functional pathing states
     */
    public PathFinder pathFinder = new PathFinder();


    /**
     * Initializes RobotState and field
     */
    public RobotState() {
        resetPosition();
        FieldConfig.setupField(field);

        simArm.setColor(new Color8Bit(125, 125, 125));
        simCollector.setLineWeight(4);
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
        actualElevatorAngleState = Elevator.ANGLE_STATE.STOW;
        actualElevatorExtensionState = Elevator.EXTENSION_STATE.MIN;
        actualCollectorRollerState = Collector.ROLLER_STATE.STOP;
        actualCollectorPivotState = Collector.PIVOT_STATE.STOW;
        isPoseUpdated = true;
        visibleTarget = new VisionPoint();
        drivetrainTemp = 0;
        vehicleToFloorProximityCentimeters = 0;
        gameElementChanged = false;
        actualElevatorAngle = 0;
        actualElevatorExtensionInches = 0;
        actualCollectorAngle = 0;
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
        return fieldToVehicle.getRotation().plus(Constants.kCameraMountingOffset.getRotation());
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
        if (RobotBase.isSimulation()) {
            // elevator
            double actEleExtMeters = (actualElevatorExtensionInches * 0.0254);
            double elevatorLength = Elevator.kElevatorMinLength + actEleExtMeters;

            simArm.setLength(elevatorLength);
            simArm.setAngle(actualElevatorAngle);

            // collector
            simCollector.setAngle(actualCollectorAngle);
            Color8Bit color;
            if (actualGameElement == Collector.GAME_ELEMENT.CONE) {
                color = new Color8Bit(255, 255, 0);
            } else if (actualGameElement == Collector.GAME_ELEMENT.CUBE) {
                color = new Color8Bit(0, 0, 255);
            } else {
                color = new Color8Bit(125, 125, 125);
            }
            simCollector.setColor(color);
            SmartDashboard.putData("Mech 2D", mechCanvas);

            // for advantagescope CAD model logging :)
            Quaternion elevatorRot = new Rotation3d(0, Math.toRadians(actualElevatorAngle), 0).getQuaternion();
            double w_rot = elevatorRot.getW();
            double x_rot = elevatorRot.getX();
            double y_rot = elevatorRot.getY();
            double z_rot = elevatorRot.getZ();

            double xExtension = Math.cos(Math.toRadians(actualElevatorAngle)) * actEleExtMeters;
            double zExtension = Math.sin(Math.toRadians(actualElevatorAngle)) * actEleExtMeters;

            SmartDashboard.putNumberArray(
                    "Elevator/3dPoses/FirstExtension",
                    new double[]{0, 0, 0, w_rot, x_rot, y_rot, z_rot}
            );
            SmartDashboard.putNumberArray(
                    "Elevator/3dPoses/SecondExtension",
                    new double[]{xExtension * 0.333, 0, zExtension * 0.333, w_rot, x_rot, y_rot, z_rot}
            );
            SmartDashboard.putNumberArray(
                    "Elevator/3dPoses/ThirdExtension",
                    new double[]{xExtension * 0.666, 0, zExtension * 0.666, w_rot, x_rot, y_rot, z_rot}
            );
            SmartDashboard.putNumberArray(
                    "Elevator/3dPoses/FourthExtension",
                    new double[]{xExtension, 0, zExtension, w_rot, x_rot, y_rot, z_rot}
            );

            // collector
            Quaternion colRot = new Rotation3d(0, Math.toRadians(actualElevatorAngle), 0)
                    .plus(new Rotation3d(0, Math.toRadians(actualCollectorAngle), 0))
                    .getQuaternion();
            w_rot = colRot.getW();
            x_rot = colRot.getX();
            y_rot = colRot.getY();
            z_rot = colRot.getZ();

            xExtension = Math.cos(Math.toRadians(actualElevatorAngle)) * actEleExtMeters + Elevator.kElevatorMinLength;
            zExtension = Math.sin(Math.toRadians(actualElevatorAngle)) * actEleExtMeters + Elevator.kElevatorMinLength;

            SmartDashboard.putNumberArray(
                    "Collector/3dPoses/Collector",
                    new double[]{xExtension, 0 , zExtension, w_rot, x_rot, y_rot, z_rot}
            );
        }
    }
}
