package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

/**
 * A class that interfaces with the Pigeon2
 */
public class Pigeon2Impl extends Pigeon2 implements IPigeonIMU {

    /**
     * Instantiates a Pigeon2Impl
     *
     * @param id     CAN-bus ID
     * @param canBus CAN-bus name (if multiple)
     */
    public Pigeon2Impl(int id, String canBus) {
        super(id, canBus);
    }

    /**
     * Returns gyroscopic yaw / transverse planar angle
     *
     * @return yaw (degrees)
     * @see IPigeonIMU#getYaw()
     */
    @Override
    public double getYaw() {
        return super.getYaw();
    }

    /**
     * Returns x, y, and z acceleration in a casted fixed point double array
     *
     * @return acceleration
     * @see IPigeonIMU#getAcceleration()
     */
    @Override
    public double[] getAcceleration() {
        short[] accel = new short[3];
        getBiasedAccelerometer(accel);
        return new double[]{accel[0], accel[1], accel[2]};
    }

    /**
     * Sets the gyroscopic yaw to a specific angle
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#getAcceleration()
     */
    @Override
    public ErrorCode setYaw(double angle) {
        return super.setYaw(angle);
    }

    /**
     * Sets the gyroscopic yaw to a specific angle
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#setFusedHeading(double)
     */
    @Override
    public ErrorCode setFusedHeading(double angle) {
        return super.setYaw(angle);
    }

    /**
     * Sets the accumulated z angle to angleDeg
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#setAccumZAngle(double)
     */
    @Override
    public ErrorCode setAccumZAngle(double angle) {
        return super.setAccumZAngle(angle);
    }

    /**
     * Returns true if a pigeon reset has occurred
     *
     * @return boolean hasResetOccurred
     * @see IPigeonIMU#hasResetOccurred()
     */
    @Override
    public boolean hasResetOccurred() {
        return super.hasResetOccurred();
    }

    /**
     * Configures factory defaults
     *
     * @return ErrorCode / void
     * @see IPigeonIMU#configFactoryDefault()
     */
    @Override
    public ErrorCode configFactoryDefault() {
        return super.configFactoryDefault();
    }
}
