package com.team1816.lib.subsystems.drive;

/**
 * Base interface for a swerve drivetrain. Consists of a list of swerve modules.
 */
public interface SwerveDrivetrain extends TrackableDrivetrain {
    ISwerveModule[] getSwerveModules();
}
