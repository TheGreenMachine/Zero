package com.team1816.lib.util.team254;

import static com.team1816.lib.subsystems.drive.Drive.kMaxAngularSpeed;
import static com.team1816.lib.subsystems.drive.Drive.kOpenLoopMaxVelMeters;

import com.team1816.lib.util.driveUtil.SwerveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Class based on Team 1323's sendInput method to make driving feel better
 */
public class SwerveDriveHelper implements DriveHelper {

    private static final double kHighAdjustmentPower = 1.50; // 1.75 + 0.4375
    private static final double kLowAdjustmentPower = 1.50;
    private static final double kMaxSpeed = (kOpenLoopMaxVelMeters);
    private static final double kMaxRotation = kMaxAngularSpeed;

    private static final double kHighPowerRotationScalar = 0.8;
    private static final double kLowPowerRotationScalar = 0.025; //yml time
    private static final double kLowPowerScalar = 0.075; //yml time
    private static final double kRotationExponent = 6.0;
    private static final double kPoleThreshold = 0.0;
    private static final double kRobotRelativePoleThreshold = Math.toRadians(5);
    private static final double kDeadband = 0.15;
    private static final double kRotationDeadband = 0.15;

    /**
     * Instantiates a SwerveDriveHelper
     */
    public SwerveDriveHelper() {}

    /**
     * Calculates a DriveSignal based on inputs with vector modulated polling
     * @param forwardInput throttle
     * @param strafeInput strafe
     * @param rotationInput rotation
     * @param low_power boolean
     * @param field_relative boolean (field-centric)
     * @param use_heading_controller boolean
     * @return SwerveDriveSignal
     * @see SwerveDriveSignal
     */
    @Override
    public SwerveDriveSignal calculateDriveSignal(
        double forwardInput,
        double strafeInput,
        double rotationInput,
        boolean low_power,
        boolean field_relative,
        boolean use_heading_controller
    ) {
        Translation2d translationalInput = new Translation2d(forwardInput, strafeInput);
        double inputMagnitude = translationalInput.getNorm();

        // Snap the translational input to its nearest pole, if it is within a certain
        // threshold of it.

        Rotation2d translationalInputDirection = new Rotation2d(
            translationalInput.getX(),
            translationalInput.getY()
        );

        if (field_relative) {
            if (
                Math.abs(
                    translationalInputDirection
                        .unaryMinus()
                        .rotateBy(nearestPole(translationalInputDirection))
                        .getRadians()
                ) <
                kPoleThreshold
            ) {
                translationalInput =
                    new Translation2d(
                        nearestPole(translationalInputDirection).getCos(),
                        nearestPole(translationalInputDirection).getSin()
                    )
                    .times(inputMagnitude);
            }
        } else {
            if (
                Math.abs(
                    translationalInputDirection
                        .unaryMinus()
                        .rotateBy(nearestPole(translationalInputDirection))
                        .getRadians()
                ) <
                kRobotRelativePoleThreshold
            ) {
                translationalInput =
                    new Translation2d(
                        nearestPole(translationalInputDirection).getCos(),
                        nearestPole(translationalInputDirection).getSin()
                    )
                    .times(inputMagnitude);
            }
        }

        if (inputMagnitude < kDeadband) {
            translationalInput = new Translation2d();
            inputMagnitude = 0;
        }

        // Scale x and y by applying a power to the magnitude of the vector they create,
        // in order to make the controls less sensitive at the lower end.
        final double power = (low_power) ? kHighAdjustmentPower : kLowAdjustmentPower;
        Rotation2d direction = translationalInputDirection;
        double scaledMagnitude = Math.pow(inputMagnitude, power);
        translationalInput =
            new Translation2d(
                direction.getCos() * scaledMagnitude,
                direction.getSin() * scaledMagnitude
            );

        rotationInput = (Math.abs(rotationInput) < kRotationDeadband) ? 0 : rotationInput;
        if (use_heading_controller) { // current constants are tuned to be put to the power of 1.75, and I don't want to retune right now
            rotationInput =
                Math.pow(Math.abs(rotationInput), 1.75) * Math.signum(rotationInput);
        } else {
            rotationInput =
                Math.pow(Math.abs(rotationInput), kRotationExponent) *
                Math.signum(rotationInput);
        }

        translationalInput = translationalInput.times(kMaxSpeed);
        rotationInput *= kMaxRotation;

        if (low_power) {
            translationalInput = translationalInput.times(kLowPowerScalar);
            rotationInput *= kLowPowerRotationScalar;
        } else {
            rotationInput *= kHighPowerRotationScalar;
        }

        return SwerveKinematics.inverseKinematics(
            translationalInput.getX(),
            translationalInput.getY(),
            rotationInput,
            field_relative
        );
    }

    /**
     * Calculates the nearest Rotation2d pole with a matrix sign function for rotation inputs
     * @param rotation Rotation2d
     * @return Rotation2d
     */
    public Rotation2d nearestPole(Rotation2d rotation) {
        double pole_sin = 0.0;
        double pole_cos = 0.0;
        if (Math.abs(rotation.getCos()) > Math.abs(rotation.getSin())) {
            pole_cos = Math.signum(rotation.getCos());
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(rotation.getSin());
        }
        return new Rotation2d(pole_cos, pole_sin);
    }
}
