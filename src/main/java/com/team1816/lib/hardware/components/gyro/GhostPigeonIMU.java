package com.team1816.lib.hardware.components.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team1816.lib.hardware.components.motor.IGreenMotor;

/**
 * This class emulates the behaviour of a Pigeon that is not physically implemented on a robot
 *
 * @see IPigeonIMU
 */
public class GhostPigeonIMU implements IPigeonIMU {

    /**
     * State
     */
    double simulatedYaw; // simulated yaw
    double simulatedPitch;
    double simulatedRoll;

    /**
     * Instantiates a free ghost pigeon
     */
    public GhostPigeonIMU(int id) {
        simulatedYaw = 0;
        simulatedPitch = 0;
        simulatedRoll = 0;
    }

    /**
     * Alternately instantiates a ghost pigeon attached to a motor
     */
    public GhostPigeonIMU(IGreenMotor motor) {
    }

    /**
     * Returns the simulatedYaw
     *
     * @return simulatedYaw
     * @see IPigeonIMU#getYaw()
     */
    @Override
    public double getYaw() {
        return simulatedYaw;
    }

    /**
     * Returns the simulatedPitch
     *
     * @return simulatedPitch
     * @see IPigeonIMU#getPitch()
     */
    @Override
    public double getPitch() {
        return simulatedPitch;
    }

    /**
     * Returns the simulatedRoll
     *
     * @return simulatedRoll
     * @see IPigeonIMU#getRoll()
     */
    @Override
    public double getRoll() {
        return simulatedRoll;
    }

    /**
     * Returns constant simulated acceleration, can be modified for other purposes
     *
     * @return simulatedAcceleration
     * @see IPigeonIMU#getAcceleration()
     */
    @Override
    public double[] getAcceleration() {
        double[] accel = new double[]{0d, 0d, 9.8d};
        return accel;
    }

    /**
     * Sets the simulated yaw to a specified value
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#setYaw(double)
     */
    @Override
    public ErrorCode setYaw(double angle) {
        simulatedYaw = angle;
        return ErrorCode.OK;
    }

    /**
     * Functionality: non-existent
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#setFusedHeading(double)
     */
    @Override
    public ErrorCode setFusedHeading(double angle) {
        return ErrorCode.OK;
    }

    /**
     * Functionality: non-existent
     *
     * @param angle (degrees)
     * @return ErrorCode / void
     * @see IPigeonIMU#setAccumZAngle(double)
     */
    @Override
    public ErrorCode setAccumZAngle(double angle) {
        return ErrorCode.OK;
    }

    /**
     * Returns if a reset has occurred
     *
     * @return boolean hasResetOccurred
     * @see IPigeonIMU#hasResetOccurred()
     */
    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    /**
     * Functionality: non-existent
     *
     * @return ErrorCode / void
     * @see IPigeonIMU#configFactoryDefault()
     */
    @Override
    public ErrorCode configFactoryDefault() {
        return ErrorCode.OK;
    }

    /**
     * Functionality: non-existent
     *
     * @return ErrorCode / void
     * @see IPigeonIMU#setStatusFramePeriod(PigeonIMU_StatusFrame, int)
     */
    @Override
    public ErrorCode setStatusFramePeriod(
        PigeonIMU_StatusFrame statusFrame,
        int periodMs
    ) {
        return ErrorCode.OK;
    }
}
