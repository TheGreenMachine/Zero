package com.team1816.lib.subsystems.drive;

/**
 * Base interface for a swerve module with isolated components for a drive motor and azimuth motor with feedback control
 */
public interface ISwerveModule {
    String getModuleName();

    // angle degrees
    double getActualAzimuth();

    double getAzimuthError();

    double getDesiredAzimuth();

    // velocity ticks/100ms
    double getActualDrive();

    double getDesiredDrive();

    double getDrivePosition();

    double getDriveError();

    // Temperature C
    double getMotorTemp();
}
