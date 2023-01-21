package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * A class that interfaces with the PigeonIMU
 */
public class PigeonIMUImpl extends PigeonIMU implements IPigeonIMU {

    /**
     * State
     */
    private long m_handle = 0l;

    /**
     * Instantiates a PigeonIMUImpl
     *
     * @param id (CAN-Bus id)
     */
    public PigeonIMUImpl(int id) {
        super(id);
        m_handle = super.getHandle();
        System.out.println("PIGEON HANDLE: " + m_handle);
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
     * Returns gyroscopic pitch / transverse lateral angle
     *
     * @return pitch (degrees)
     * @see IPigeonIMU#getPitch()
     */
    @Override
    public double getPitch() {
        return super.getPitch();
    }

    /**
     * Returns gyroscopic roll / transverse frontal angle
     *
     * @return roll (degrees)
     * @see IPigeonIMU#getRoll()
     */
    @Override
    public double getRoll() {
        return super.getRoll();
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
