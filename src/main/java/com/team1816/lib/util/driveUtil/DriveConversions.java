package com.team1816.lib.util.driveUtil;

import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveModule;
import com.team1816.lib.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import static com.team1816.lib.subsystems.drive.Drive.*;

/**
 * Utility class and constants dump for Drivetrain conversions and miscellaneous math
 */
public class DriveConversions {

    private static final double azimuthPPR = SwerveModule.ModuleConfig.kAzimuthPPR;
    private static final double drivePPR = Drive.driveEncPPR;

    public static double ticksToMeters(double ticks) {
        return ticks / drivePPR * kWheelCircumferenceMeters;
    }

    public static double metersToTicks(double meters) {
        return meters / Math.PI * azimuthPPR / kWheelCircumferenceMeters;
    }

    public static double convertTicksToRadians(double ticks) {
        return ticks / azimuthPPR * 2 * Math.PI;
    }

    public static double convertRadiansToTicks(double radians) {
        return radians / (Math.PI * 2) * azimuthPPR;
    }

    public static double convertTicksToDegrees(double ticks) {
        return Units.radiansToDegrees(convertTicksToRadians(ticks));
    }

    public static double convertDegreesToTicks(double degrees) {
        return convertRadiansToTicks(Units.degreesToRadians(degrees));
    }

    public static double rotationsToMeters(double rotations) {
        return rotations * (kWheelCircumferenceMeters);
    }

    public static double metersToRotations(double meters) {
        return meters / kWheelCircumferenceMeters;
    }

    public static double metersPerSecondToTicksPer100ms(double meters_per_second) {
        return metersToRotations(meters_per_second) * drivePPR / 10.0;
    }

    public static double ticksPer100msToMetersPerSecond(double ticks_per_second) {
        return rotationsToMeters(ticks_per_second / drivePPR) * 10.0;
    }

    public static double ticksPer100MSToMPS(double ticksPer100MS) { // ticks/100ms to meters / second
        return rotationsToMeters(ticksPer100MS / drivePPR) * 10.0;
    }

    public static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * drivePPR / 10.0;
    }

    public static boolean epsilonEquals(
        final Pose2d reference,
        final Pose2d other,
        double epsilon
    ) {
        return (
            Util.epsilonEquals(reference.getX(), other.getX(), epsilon) &&
                Util.epsilonEquals(reference.getY(), other.getY(), epsilon) &&
                Util.epsilonEquals(
                    reference.getRotation().getDegrees(),
                    other.getRotation().getDegrees(),
                    epsilon
                )
        );
    }
}
