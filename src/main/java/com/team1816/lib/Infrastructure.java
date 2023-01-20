package com.team1816.lib;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.hardware.components.sensor.ProximitySensor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;

import java.util.ArrayList;
import java.util.List;

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
    public static List<ProximitySensor> proximitySensors;

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

        var frontLeft = new ProximitySensor("FLProximitySensor", 0);
        var frontRight = new ProximitySensor("FRProximitySensor", 1);
        var rearLeft = new ProximitySensor("RLProximitySensor3", 2);
        var rearRight = new ProximitySensor("RRProximitySensor4", 3);
        proximitySensors = List.of(frontLeft, frontRight, rearLeft, rearRight);
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
     * Returns the gyroscopic pitch of the pigeon
     * @return pitch
     * @see IPigeonIMU#getPitch()
     */
    public double getPitch() {
        return pigeon.getPitch();
    }

    /**
     * Returns the gyroscopic roll of the pigeon
     * @return roll
     * @see IPigeonIMU#getRoll()
     */
    public double getRoll() {
        return pigeon.getRoll();
    }

    /**
     * Returns the field-centric pitch of the pigeon
     * @return pitch
     * @see IPigeonIMU#getPitch()
     */
    public double getFieldCentricPitch() {
        Rotation3d angularState = new Rotation3d(Units.degreesToRadians(getYaw()), Units.degreesToRadians(getPitch()), Units.degreesToRadians(getRoll()));
        Rotation3d yawState = new Rotation3d(-Units.degreesToRadians(getYaw()), 0, 0);
        return Units.radiansToDegrees(angularState.rotateBy(yawState).getY());
    }

    /**
     * Returns the field-centric roll of the pigeon
     * @return roll
     * @see IPigeonIMU#getRoll()
     */
    public double getFieldCentricRoll() {
        Rotation3d angularState = new Rotation3d(Units.degreesToRadians(getYaw()), Units.degreesToRadians(getPitch()), Units.degreesToRadians(getRoll()));
        Rotation3d yawState = new Rotation3d(-Units.degreesToRadians(getYaw()), 0, 0);
        return Units.radiansToDegrees(angularState.rotateBy(yawState).getZ());
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

    /**
     * Returns the maximal locus proximity of the drivetrain to the floor
     * @return maximumProximity
     */
    public double getMaximumProximity() {
        double maximumProximity = -1;
        for (int i = 0; i<3; i++) {
            double proximity = proximitySensors.get(i).getProximity();
            if (proximity > maximumProximity && proximity < 80) {
                maximumProximity = proximity;
            }
        }
        return maximumProximity;
    }
}
