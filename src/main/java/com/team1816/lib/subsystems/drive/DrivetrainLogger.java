package com.team1816.lib.subsystems.drive;

import com.team1816.lib.subsystems.Subsystem;

/**
 * Class solely meant for logging drivetrain topics using the Badlogs logging utility.
 */
public class DrivetrainLogger {

    public static void init(TrackableDrivetrain drivetrain) {
        var isSwerve = drivetrain instanceof SwerveDrivetrain;
        var subsystem = (Subsystem) drivetrain;
        if (isSwerve) {
            for (
                int i = 0;
                i < ((SwerveDrivetrain) drivetrain).getSwerveModules().length; // 1;
                i++
            ) {
                var module = ((SwerveDrivetrain) drivetrain).getSwerveModules()[i];
                var name = module.getModuleName();
                var prefix = "Drivetrain/" + name;
                // Azimuth
                subsystem.createBadLogTopic(
                    prefix + "AzimuthPosition",
                    "ticks",
                    module::getActualAzimuth,
                    "hide",
                    "join:Drivetrain/AzimuthPosition"
                );
                subsystem.createBadLogTopic(
                    prefix + "AzimuthDemand",
                    "ticks",
                    module::getDesiredAzimuth,
                    "hide",
                    "join:Drivetrain/AzimuthPosition"
                );
                subsystem.createBadLogTopic(
                    prefix + "AzimuthError",
                    "ticks",
                    module::getAzimuthError,
                    "hide",
                    "join:Drivetrain/AzimuthError"
                );
                subsystem.createBadLogTopic(
                    prefix + "NormalizedAzimuthDemand",
                    "ticks",
                    module::getDesiredNormalizedAzimuth,
                    "hide",
                    "join:Drivetrain/NormalizedAzimuthDemand"
                );
                subsystem.createBadLogTopic(
                    prefix + "NormalizedAzimuthPosition",
                    "ticks",
                    module::getActualNormalizedAzimuth,
                    "hide",
                    "join:Drivetrain/NormalizedAzimuthPosition"
                );

                // Drive
                subsystem.createBadLogTopic(
                    prefix + "DriveVelocity",
                    "ticks",
                    module::getActualDrive,
                    "hide",
                    "join:Drivetrain/DriveVelocity"
                );
                subsystem.createBadLogTopic(
                    prefix + "DriveVelocityDemand",
                    "ticks",
                    module::getDesiredDrive,
                    "hide",
                    "join:Drivetrain/DriveVelocity"
                );
                subsystem.createBadLogTopic(
                    prefix + "DriveError",
                    "ticks",
                    module::getDriveError,
                    "hide",
                    "join:Drivetrain/DriveError"
                );
                subsystem.createBadLogTopic(
                    prefix + "DriveTemperature",
                    "degrees C",
                    module::getMotorTemp,
                    "hide",
                    "join:Drivetrain/Temperature"
                );
            }
        } else {
            subsystem.createBadLogTopic(
                "Drivetrain/LeftActVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftVelocityTicksActual,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.createBadLogTopic(
                "Drivetrain/RightActVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightVelocityTicksActual,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.createBadLogTopic(
                "Drivetrain/LeftVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftVelocityTicksDemand,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.createBadLogTopic(
                "Drivetrain/RightVel",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightVelocityTicksDemand,
                "hide",
                "join:Drivetrain/Velocities"
            );
            subsystem.createBadLogTopic(
                "Drivetrain/LeftError",
                "NativeUnits",
                ((TankDrive) drivetrain)::getLeftError,
                "hide",
                "join:Drivetrain/VelocityError"
            );
            subsystem.createBadLogTopic(
                "Drivetrain/RightError",
                "NativeUnits",
                ((TankDrive) drivetrain)::getRightError,
                "hide",
                "join:Drivetrain/VelocityError"
            );
        }
        subsystem.createBadLogTopic(
            "Drivetrain/X Desired",
            "Inches",
            drivetrain::getFieldDesiredXDisplacement,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.createBadLogTopic(
            "Drivetrain/Y Desired",
            "Inches",
            drivetrain::getFieldDesiredYDisplacement,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.createBadLogTopic(
            "Drivetrain/X Actual",
            "Inches",
            drivetrain::getFieldXDisplacement,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.createBadLogTopic(
            "Drivetrain/Y Actual",
            "Inches",
            drivetrain::getFieldYDisplacement,
            "hide",
            "join:Drivetrain/Distance"
        );
        subsystem.createBadLogTopic(
            "Drivetrain/ActualHeading",
            "Angle",
            drivetrain::getActualHeadingDegrees,
            "hide",
            "join:Drivetrain/Heading"
        );
        subsystem.createBadLogTopic(
            "Drivetrain/DesiredHeading",
            "Angle",
            drivetrain::getDesiredHeadingDegrees,
            "hide",
            "join:Drivetrain/Heading"
        );
    }
}
