package com.team1816.lib;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Super-system housing compressor, pigeon, and power distribution
 */

@Singleton
public class Infrastructure {

    /**
     * Components
     */
    private static ICompressor compressor;
    private static IPigeonIMU pigeon;
    private static PowerDistribution pd;

    private double xAccel, yAccel, zAccel;
    private double loopCount;

    private static boolean compressorEnabled;
    private static boolean compressorIsOn = false;

    /**
     * Instantiates the infrastructure with RobotFactory
     * @param factory RobotFactory
     * @see RobotFactory
     */
    @Inject
    public Infrastructure(RobotFactory factory) {
        compressor = factory.getCompressor();
        pigeon = factory.getPigeon();
        pd = factory.getPd();
        compressorEnabled = factory.isCompressorEnabled();

        xAccel = yAccel = zAccel = loopCount = 0;
    }

    /**
     * Starts the compressor. Not used now because the compressor turns on by default once the robot is enabled.
     */
    public void startCompressor() {
        if (compressorEnabled && !compressorIsOn) {
            compressor.enableDigital();
            compressorIsOn = true;
        }
    }

    /**
     * Stops the compressor
     * @see Infrastructure#startCompressor()
     */
    public void stopCompressor() {
        if (compressorEnabled && compressorIsOn) {
            compressor.disable();
            compressorIsOn = false;
        }
    }

    /**
     * Resets the pigeon gyroscope based on a Rotation2d
     * @param angle Rotation2d
     */
    public void resetPigeon(Rotation2d angle) {
        System.out.println("resetting Pigeon");
        pigeon.setYaw(angle.getDegrees());
    }

    /**
     * Returns the pigeon associated with the infrastructure
     * @return IPigeonIMU
     * @see IPigeonIMU
     */
    public IPigeonIMU getPigeon() {
        return pigeon;
    }

    /**
     * Returns the gyroscopic yaw of the pigeon
     * @return yaw
     * @see IPigeonIMU#getYaw()
     */
    public double getYaw() {
        return pigeon.getYaw();
    }

    /**
     * Updates accelerometer readings from the pigeon on a timed loop basis
     * @see IPigeonIMU#getAcceleration()
     */
    public void update() {
        if (loopCount > 5) {
            loopCount = 0;
            double[] accel = pigeon.getAcceleration();
            xAccel = accel[0] * Constants.gravitationalAccelerationConstant / 16384;
            yAccel = accel[1] * Constants.gravitationalAccelerationConstant / 16384;
            zAccel = -accel[2] * Constants.gravitationalAccelerationConstant / 16384;
        } else {
            loopCount++;
        }
    }

    /**
     * Returns the cached acceleration
     * @return accel
     */
    public Double[] getAcceleration() {
        return new Double[] { xAccel, yAccel, zAccel };
    }

    /**
     * Returns acceleration x-axis
     * @return xAccel
     */
    public double getXAcceleration() {
        return getAcceleration()[0];
    }

    /**
     * Returns acceleration y-axis
     * @return yAccel
     */
    public double getYAcceleration() {
        return getAcceleration()[1];
    }

    /**
     * Returns acceleration z-axis
     * @return zAccel
     */
    public double getZAcceleration() {
        return getAcceleration()[2];
    }

    /**
     * Returns the power distribution associated with the Infrastructure
     * @return PowerDistribution
     * @see PowerDistribution
     */
    public PowerDistribution getPd() {
        return pd;
    }

    /**
     * Emulates gyroscope behaviour of the pigeon in simulation environments
     * @param radianOffsetPerLoop loop ratio
     * @param gyroDrift drift
     */
    public void simulateGyro(double radianOffsetPerLoop, double gyroDrift) {
        pigeon.setYaw(getYaw() + radianOffsetPerLoop + gyroDrift);
    }
}
