package com.team1816.lib;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.season.configuration.Constants;
import com.team1816.season.hardware.components.ProxySensor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;

import java.util.ArrayList;

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


    private static boolean compressorEnabled;
    private static boolean compressorIsOn = false;

    public static ProxySensor frontLeft, frontRight, backLeft, backRight;

    public static String maxProxyName;

    public static double threshold;

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
        //Sensor implementation
        frontLeft = new ProxySensor("proxySensor1", 0);
        frontRight = new ProxySensor("proxySensor2", 1);
        backLeft = new ProxySensor("proxySensor3", 2);
        backRight = new ProxySensor("proxySensor4", 3);
        threshold = factory.getConstant("proxyDistanceThreshold");
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

    public double getMaxDistance(){
        ArrayList<ProxySensor> distances = new ArrayList <ProxySensor>();
        distances.add(frontLeft);
        distances.add(frontRight);
        distances.add(backLeft);
        distances.add(backRight);
        double maxValue = -1;
        for(int i = 0; i<3; i++){
            if(distances.get(i).getDistance()>80){
                System.out.println(distances.get(i).getName() + " is getting life force SUCKED");
                continue;
            } else if(distances.get(i).getDistance()>maxValue){
                maxValue = distances.get(i).getDistance();
                maxProxyName = distances.get(i).getName();
            }
        }

        //for testing/sout purposes
        System.out.println("sensor 1: " + frontLeft.getDistance() + " sensor 2: " +
            frontRight.getDistance() + " sensor 3: " + backLeft.getDistance() + " sensor 4: " +
            backRight.getDistance() + "  maxDistance: " + maxValue + "   from " +
            maxProxyName);
        return maxValue;
    }


    public boolean crossDistanceThreshold(){
        return getMaxDistance() > threshold;
    }
}
