package com.team1816.lib.util.team254;

/**
 * Base interface for control modulation drive helpers
 */
public interface DriveHelper {
    SwerveDriveSignal calculateDriveSignal(
        double forwardInput,
        double strafeInput,
        double rotationInput,
        boolean low_power,
        boolean field_relative,
        boolean use_heading_controller
    );
}
