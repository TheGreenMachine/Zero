package com.team1816.season.subsystems;

import com.team1816.TestUtil;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.Before;
import org.junit.Test;
import org.mockito.stubbing.OngoingStubbing;

import static com.team1816.lib.subsystems.drive.Drive.*;
import static com.team1816.lib.subsystems.drive.SwerveDrive.swerveKinematics;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

// @RunWith(JUnit4.class)
public class SwerveDriveTest {

    private final RobotState state;
    private final Infrastructure mInfra;
    private final SwerveDrive mDrive;
    private final double maxVel = 2.54; //  m per sec
    private final double maxRotVel = 2 * Math.PI; // rad per sec;
    private final Drive.Factory mDriveFactory;
    private static OngoingStubbing<Object> mockRobot;

    public SwerveDriveTest() {
        mInfra = mock(Infrastructure.class);
        when(mInfra.getPigeon()).thenReturn(mock(IPigeonIMU.class));
        Injector.register(mInfra);
        RobotFactory mockFactory = mock(RobotFactory.class);
        when(mockFactory.getConstant("maxVelOpenLoop")).thenReturn(maxVel);
        when(mockFactory.getConstant("maxRotVel")).thenReturn(maxRotVel);
        TestUtil.SetupMockRobotFactory(mockFactory);
        mDriveFactory = mock(Drive.Factory.class);
        Injector.register(mDriveFactory);
        var test = mDriveFactory.getClass();
        state = Injector.get(RobotState.class);
        mDrive = Injector.get(SwerveDrive.class);
    }

    @Before
    public void setUp() {
        when(mDriveFactory.getInstance()).thenReturn(mDrive);
        mDrive.zeroSensors();
        state.resetPosition();
    }

    private SwerveModuleState[] getExpectedState(
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        Rotation2d robotAngle
    ) {
        // TODO remove conversion when constants class is converted to metric
        var states = swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                robotAngle
            )
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVel);
        return states;
    }

    @Test
    public void testFactoryMock() {
        assertEquals(maxVel, kMaxVelOpenLoopMeters, .01);
        assertEquals(maxRotVel, kMaxAngularSpeed, .01);
        assertEquals(kDriveWheelTrackWidthMeters, kDriveWheelbaseLengthMeters, .01);
    }

    @Test
    public void testTeleopRotation() {
        mDrive.setTeleopInputs(0, 0, 1);
        mDrive.writeToHardware();
        mDrive.readFromHardware();
        verifyStates(mDrive.getStates(), 0, 0, maxRotVel);
    }

    @Test
    public void testTeleopForward() {
        mDrive.setTeleopInputs(1, 0, 0);
        mDrive.writeToHardware();
        mDrive.readFromHardware();
        verifyStates(mDrive.getStates(), maxVel, 0, 0);
    }

    @Test
    public void testTeleopStrafe() {
        mDrive.setTeleopInputs(0, 1, 0);
        mDrive.writeToHardware();
        mDrive.readFromHardware();
        verifyStates(mDrive.getStates(), 0, maxVel, 0);
    }

    public void verifyStates(
        SwerveModuleState[] states,
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond
    ) {
        var expected = getExpectedState(
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadiansPerSecond,
            Constants.EmptyRotation2d
        );

        // We verify the returned value from getState to match the original value.
        // So even though we are percent output the getState is used for feedback
        // this needs to be real velocity values that are returned
        for (int i = 0; i < states.length; i++) {
            var actVel = states[i].speedMetersPerSecond;
            assertEquals(
                "Velocity does not match",
                Math.abs(expected[i].speedMetersPerSecond),
                Math.abs(actVel),
                .01
            );
            var actRot = states[i].angle.getRadians();
            var expRot = expected[i].angle.getRadians();
            assertEquals("Rotation does not match", expRot, actRot, .2);
        }
    }
}
