package com.team1816.lib.subsystems.drive;

/**
 * Base interface for any drivetrain that has an odometry and kinematics that can be tracked relative to its initial position
 */
public interface TrackableDrivetrain {
    double getFieldXDisplacement();

    double getFieldYDisplacement();

    double getFieldDesiredXDisplacement();

    double getFieldDesiredYDisplacement();

    double getActualHeadingDegrees();

    double getDesiredHeadingDegrees();
}
