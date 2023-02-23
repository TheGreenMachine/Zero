package com.team1816.lib.subsystems.drive;

import badlog.lib.BadLog;
import com.team1816.season.configuration.Constants;

/**
 * Class solely meant for logging drivetrain topics using the Badlogs logging utility.
 */
public class DrivetrainLogger {

    public static void init(TrackableDrivetrain drivetrain) {
        var isSwerve = drivetrain instanceof SwerveDrivetrain;
        if(Constants.kIsBadlogEnabled && Constants.kIsLoggingDrivetrain){
            if (isSwerve) {
                for (
                        int i = 0;
                        i < 1; //((SwerveDrivetrain) drivetrain).getSwerveModules().length;
                        i++
                ) {
                    var module = ((SwerveDrivetrain) drivetrain).getSwerveModules()[i];
                    var name = module.getModuleName();
                    var prefix = "Drivetrain/" + name;
                    // Azimuth
                    BadLog.createTopic(
                            prefix + "AzimuthPosition",
                            "ticks",
                            module::getActualAzimuth,
                            "hide",
                            "join:Drivetrain/AzimuthPosition"
                    );
                    BadLog.createTopic(
                            prefix + "AzimuthDemand",
                            "ticks",
                            module::getDesiredAzimuth,
                            "hide",
                            "join:Drivetrain/AzimuthPosition"
                    );
                    BadLog.createTopic(
                            prefix + "AzimuthError",
                            "ticks",
                            module::getAzimuthError,
                            "hide",
                            "join:Drivetrain/AzimuthError"
                    );

                    // Drive
                    BadLog.createTopic(
                            prefix + "DriveVelocity",
                            "ticks",
                            module::getActualDrive,
                            "hide",
                            "join:Drivetrain/DriveVelocity"
                    );
                    BadLog.createTopic(
                            prefix + "DriveVelocityDemand",
                            "ticks",
                            module::getDesiredDrive,
                            "hide",
                            "join:Drivetrain/DriveVelocity"
                    );
                    BadLog.createTopic(
                            prefix + "DriveError",
                            "ticks",
                            module::getDriveError,
                            "hide",
                            "join:Drivetrain/DriveError"
                    );
                    BadLog.createTopic(
                            prefix + "DriveTemperature",
                            "degrees C",
                            module::getMotorTemp,
                            "hide",
                            "join:Drivetrain/Temperature"
                    );
                }
            } else {
                BadLog.createTopic(
                        "Drivetrain/LeftActVel",
                        "NativeUnits",
                        ((TankDrive) drivetrain)::getLeftVelocityTicksActual,
                        "hide",
                        "join:Drivetrain/Velocities"
                );
                BadLog.createTopic(
                        "Drivetrain/RightActVel",
                        "NativeUnits",
                        ((TankDrive) drivetrain)::getRightVelocityTicksActual,
                        "hide",
                        "join:Drivetrain/Velocities"
                );
                BadLog.createTopic(
                        "Drivetrain/LeftVel",
                        "NativeUnits",
                        ((TankDrive) drivetrain)::getLeftVelocityTicksDemand,
                        "hide",
                        "join:Drivetrain/Velocities"
                );
                BadLog.createTopic(
                        "Drivetrain/RightVel",
                        "NativeUnits",
                        ((TankDrive) drivetrain)::getRightVelocityTicksDemand,
                        "hide",
                        "join:Drivetrain/Velocities"
                );
                BadLog.createTopic(
                        "Drivetrain/LeftError",
                        "NativeUnits",
                        ((TankDrive) drivetrain)::getLeftError,
                        "hide",
                        "join:Drivetrain/VelocityError"
                );
                BadLog.createTopic(
                        "Drivetrain/RightError",
                        "NativeUnits",
                        ((TankDrive) drivetrain)::getRightError,
                        "hide",
                        "join:Drivetrain/VelocityError"
                );
            }
            BadLog.createTopic(
                    "Drivetrain/X Desired",
                    "Inches",
                    drivetrain::getFieldDesiredXDisplacement,
                    "hide",
                    "join:Drivetrain/Distance"
            );
            BadLog.createTopic(
                    "Drivetrain/Y Desired",
                    "Inches",
                    drivetrain::getFieldDesiredYDisplacement,
                    "hide",
                    "join:Drivetrain/Distance"
            );
            BadLog.createTopic(
                    "Drivetrain/X Actual",
                    "Inches",
                    drivetrain::getFieldXDisplacement,
                    "hide",
                    "join:Drivetrain/Distance"
            );
            BadLog.createTopic(
                    "Drivetrain/Y Actual",
                    "Inches",
                    drivetrain::getFieldYDisplacement,
                    "hide",
                    "join:Drivetrain/Distance"
            );
            BadLog.createTopic(
                    "Drivetrain/ActualHeading",
                    "Angle",
                    drivetrain::getActualHeadingDegrees,
                    "hide",
                    "join:Drivetrain/Heading"
            );
            BadLog.createTopic(
                    "Drivetrain/DesiredHeading",
                    "Angle",
                    drivetrain::getDesiredHeadingDegrees,
                    "hide",
                    "join:Drivetrain/Heading"
            );
        }
    }
}
