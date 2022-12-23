package com.team1816.season.subsystems;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.team1816.TestUtil;
import com.team1816.lib.Injector;
import com.team1816.lib.hardware.components.motor.GhostMotor;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.turret.Turret;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Spy;

// @RunWith(JUnit4.class)
public class TurretTest {

    private final RobotState state;
    private final RobotFactory mockFactory;
    private Turret mTurret;
    private double encTickSouth = 1980;
    private double encTick45 = encTickSouth + 512;
    private double encTick315 = encTickSouth - 512;
    private final double encPPR = 4096;

    @Spy
    private Constants constants;

    public TurretTest() {
        mockFactory = mock(RobotFactory.class);
        when(mockFactory.getConstant(Turret.NAME, "absPosTicksSouth"))
            .thenReturn(encTickSouth);
        when(mockFactory.getConstant(Turret.NAME, "turretPPR")).thenReturn(encPPR);
        when(mockFactory.getConstant(Turret.NAME, "encPPR")).thenReturn(encPPR);
        TestUtil.SetupMockRobotFactory(mockFactory);
        state = Injector.get(RobotState.class);
    }

    @Before
    public void setUp() {
        mTurret = Injector.get(Turret.class);
        mTurret.zeroSensors();
        state.resetPosition();
    }

    @Test
    public void fieldFollowingTest() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        mTurret.writeToHardware();
        mTurret.readFromHardware();
        assertEquals(0, state.getLatestFieldToTurret().getDegrees(), 0.1);
        assertEquals(0, state.vehicleToTurret.getDegrees(), .01);
        assertEquals(encTickSouth, mTurret.getActualPosTicks(), .01);
    }

    @Test
    public void fieldFollowing45Test() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        state.fieldToVehicle = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
        mTurret.writeToHardware();
        mTurret.readFromHardware();
        assertEquals(45, state.vehicleToTurret.getDegrees(), .01);
        assertEquals(0, state.getLatestFieldToTurret().getDegrees(), 0.1);
        // Turret should move CW
        assertEquals(encTick45, mTurret.getActualPosTicks(), .01);
    }

    @Test
    public void fieldFollowing315Test() {
        mTurret.setTurretAngle(0);
        mTurret.setControlMode(Turret.ControlMode.FIELD_FOLLOWING);
        state.fieldToVehicle = new Pose2d(0, 0, Rotation2d.fromDegrees(-45));
        mTurret.writeToHardware();
        mTurret.readFromHardware();
        assertEquals(315, state.vehicleToTurret.getDegrees(), .01);
        assertEquals(0, state.getLatestFieldToTurret().getDegrees(), 0.1);
        // Turret should move CCW
        assertEquals(encTick315, mTurret.getActualPosTicks(), .01);
    }

    @Test
    public void fieldFollowingDoubleTest() {
        setupDoubleRotation();
        fieldFollowingTest();
    }

    @Test
    public void fieldFollowing45DoubleTest() {
        setupDoubleRotation();
        fieldFollowing45Test();
    }

    @Test
    public void fieldFollowing315DoubleTest() {
        setupDoubleRotation();
        fieldFollowing315Test();
    }

    @Test
    public void convertTurretDegreesToTicksTest() {
        assertEquals(encTickSouth, mTurret.convertTurretDegreesToTicks(0), .01);
        assertEquals(encTickSouth, mTurret.convertTurretDegreesToTicks(360), .01);
        assertEquals(encTickSouth, mTurret.convertTurretDegreesToTicks(720), .01);
        assertEquals(encTick45, mTurret.convertTurretDegreesToTicks(45), .01);
        assertEquals(encTick315, mTurret.convertTurretDegreesToTicks(-45), .01);
    }

    @Test
    public void convertTurretTicksToDegrees() {
        assertEquals(0, mTurret.convertTurretTicksToDegrees(encTickSouth), .01);
        assertEquals(45, mTurret.convertTurretTicksToDegrees(encTick45), .01);
        assertEquals(315, mTurret.convertTurretTicksToDegrees(encTick315), .01);
    }

    @Test
    public void convertTurretTicksToDegreesDoubleRotation() {
        setupDoubleRotation();
        convertTurretTicksToDegrees();
    }

    @Test
    public void convertTurretDegreesToTicksDoubleRotationTest() {
        setupDoubleRotation();
        convertTurretDegreesToTicksTest();
    }

    private void setupDoubleRotation() {
        encTickSouth = 1980 * 2;
        encTick45 = encTickSouth + 1024;
        encTick315 = encTickSouth - 1024;
        when(mockFactory.getConstant(Turret.NAME, "absPosTicksSouth"))
            .thenReturn(encTickSouth);
        when(mockFactory.getConstant(Turret.NAME, "turretPPR")).thenReturn(encPPR * 2);
        mTurret = Injector.get(Turret.class);
        mTurret.zeroSensors();
        state.resetPosition();
    }

    @Test
    public void zeroSensors0Test() {
        zeroSensorsTest(0, 53248.0);
    }

    @Test
    public void zeroSensors2048Test() {
        zeroSensorsTest(2048, 53248.0);
    }

    @Test
    public void zeroSensors2048SingleRotationTest() {
        zeroSensorsTest(2048, 4096);
    }

    @Test
    public void zeroSensors4095Test() {
        zeroSensorsTest(4095, 53248.0);
    }

    public void zeroSensorsTest(int absInit, double turretPPR) {
        when(mockFactory.getConstant(Turret.NAME, "turretPPR")).thenReturn(turretPPR);
        when(mockFactory.getMotor(Turret.NAME, "turret"))
            .thenReturn(new GhostMotor(0, absInit, "turret"));
        mTurret = Injector.get(Turret.class);
        mTurret.zeroSensors();
        Assert.assertEquals(
            mTurret.kTurretPPR / 2.0 - mTurret.kTurretPPR == mTurret.kAbsPPR
                ? 0
                : absInit,
            mTurret.getActualPosTicks(),
            1
        );
    }
}
