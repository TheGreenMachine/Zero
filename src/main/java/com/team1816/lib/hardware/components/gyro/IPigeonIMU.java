package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

/**
 * The root interface for the CTRE Pigeon component
 */
public interface IPigeonIMU {
    /**
     * Returns gyroscopic yaw / transverse planar angle
     *
     * @return yaw (degrees)
     */
    double getYaw();

    /**
     * Returns gyroscopic pitch / transverse lateral angle
     *
     * @return pitch (degrees)
     */
    double getPitch();

    /**
     * Returns gyroscopic roll / transverse frontal angle
     *
     * @return roll (degrees)
     */
    double getRoll();

    /**
     * Returns x, y, and z acceleration in a casted fixed point double array
     *
     * @return acceleration
     */
    double[] getAcceleration();

    /**
     * Sets the gyroscopic yaw to a specific angle
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     */
    ErrorCode setYaw(double angle);

    /**
     * Same as setYaw();
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#setYaw(double)
     */
    ErrorCode setFusedHeading(double angle);

    /**
     * Sets the accumulated z angle to angleDeg
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     */
    ErrorCode setAccumZAngle(double angle);

    /**
     * Returns true if a pigeon reset has occurred
     *
     * @return hasResetOccurred
     */
    boolean hasResetOccurred();

    /**
     * Configures factory defaults
     *
     * @return ErrorCode / void
     */
    ErrorCode configFactoryDefault();

    /**
     * Sets the synchronized status frame period of the pigeon and is directly related to CAN-bus utilization
     *
     * @param statusFrame
     * @param periodMs
     * @return
     */
    ErrorCode setStatusFramePeriod(PigeonIMU_StatusFrame statusFrame, int periodMs);
}
