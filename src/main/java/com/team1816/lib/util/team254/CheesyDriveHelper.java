package com.team1816.lib.util.team254;

import com.team1816.lib.util.Util;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that the "turning" stick controls the curvature
 * of the robot's path rather than its rate of heading change. This helps make the robot more controllable at high
 * speeds. Also handles the robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
 * turn-in-place maneuvers.
 */
public class CheesyDriveHelper implements DriveHelper {

    private static final double kThrottleDeadband = 0.035; // 0.035
    private static final double kWheelDeadband = 0.02; // 0.02, 0.01

    // These factor determine how fast the wheel traverses the "non linear" sine curve.
    private static final double kHighWheelNonLinearity = 0.01;
    private static final double kLowWheelNonLinearity = 0.5;

    private static final double kHighNegInertiaScalar = 0.0;

    private static final double kLowNegInertiaThreshold = 0.65;
    private static final double kLowNegInertiaTurnScalar = 3.0;
    private static final double kLowNegInertiaCloseScalar = 3.0;
    private static final double kLowNegInertiaFarScalar = 4.0;

    private static final double kHighSensitivity = 0.6;
    private static final double kLowSensitiity = 0.469; // 0.625

    private static final double kWheelQuckTurnScalar = .48; // .65

    private static final double kQuickStopDeadband = 0.5;
    private static final double kQuickStopWeight = 0.125;
    private static final double kQuickStopScalar = 2.8;
    private static final double SET_SPEED_DIFF_MAX = 0.056;

    private double mOldWheel = 0.0;
    private double mQuickStopAccumlator = 0.0;
    private double mNegInertiaAccumlator = 0.0;

    private double leftPrevPwm = 0;
    private double rightPrevPwm = 0;

    private static final double throttleRate = 1.0;
    private static final double turnRate = 2.5;

    private static final SlewRateLimiter throttleFilter = new SlewRateLimiter(throttleRate);
    private static final SlewRateLimiter turnFilter = new SlewRateLimiter(turnRate);

    /**
     * Returns a modified DriveSignal for an arcade style control path
     *
     * @param throttle throttle
     * @param wheel    rotation
     * @return DriveSignal
     * @see DriveSignal
     */
    public DriveSignal arcadeDrive(double throttle, double wheel) {
        return null;
    }

    /**
     * Generates a modified DriveSignal for a tank drive with inertial scaling and point power management based on inputs
     *
     * @param throttle    throttle
     * @param wheel       rotation
     * @param isQuickTurn boolean
     * @param isHighGear  boolean for an optional transmission
     * @return DriveSignal
     * @see DriveSignal
     */
    public DriveSignal cheesyDrive(
        double throttle,
        double wheel,
        boolean isQuickTurn,
        boolean isHighGear
    ) {
        wheel = throttleFilter.calculate(handleDeadband(wheel, kWheelDeadband));
        throttle = turnFilter.calculate(handleDeadband(throttle, kThrottleDeadband));

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = kHighWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        } else {
            wheelNonLinearity = kLowWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (wheel * negInertia > 0) {
            // If we are moving away from 0.0, aka, trying to get more wheel.
            negInertiaScalar = kLowNegInertiaTurnScalar;
        } else {
            // Otherwise, we are attempting to go back to 0.0.
            if (Math.abs(wheel) > kLowNegInertiaThreshold) {
                negInertiaScalar = kLowNegInertiaFarScalar;
            } else {
                negInertiaScalar = kLowNegInertiaCloseScalar;
            }
        }

        sensitivity = kLowSensitiity;

        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle; // Math.signum(throttle) * (throttle * throttle);

        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < kQuickStopDeadband) {
                double alpha = kQuickStopWeight;
                mQuickStopAccumlator =
                    (1 - alpha) *
                        mQuickStopAccumlator +
                        alpha *
                            Util.limit(wheel, 1.0) *
                            kQuickStopScalar;
            }
            overPower = 1.0;
            angularPower = wheel * kWheelQuckTurnScalar;
        } else {
            overPower = 0.0;
            angularPower = 0.75 * (wheel) * sensitivity - mQuickStopAccumlator; // removed variable throttle input to calculation
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }

        return new DriveSignal(leftPwm, rightPwm);
    }

    /**
     * Alternatively generates a DriveSignal based on inputs
     *
     * @param throttle    throttle
     * @param wheel       rotation
     * @param isQuickTurn boolean
     * @return DriveSignal
     * @see DriveSignal
     */
    public DriveSignal cheesyDrive(double throttle, double wheel, boolean isQuickTurn) {
        return cheesyDrive(throttle, wheel, isQuickTurn, false);
    }

    /**
     * Modulates demand at a deadband value
     *
     * @param val      input
     * @param deadband deadband
     * @return output
     */
    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    /**
     * Interface inherited method for mapping to a swerve drive signal
     *
     * @param forwardInput           throttle
     * @param strafeInput            strafe
     * @param rotationInput          rotation
     * @param low_power              boolean
     * @param field_relative         boolean (field-centric)
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
        return null;
    }
}
