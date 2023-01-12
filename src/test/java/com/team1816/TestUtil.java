package com.team1816;

import com.team1816.lib.Injector;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.drive.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class TestUtil {

    public static void SetupMockRobotFactory(RobotFactory mockFactory) {
        if (mockFactory == null) mockFactory = mock(RobotFactory.class);
        // any generic calls from Constants typically
        when(mockFactory.getConstant(anyString(), anyDouble()))
            .thenAnswer(input -> input.getArguments()[1]);
        //for infrastructure
        when(mockFactory.getCompressor()).thenReturn(mock(ICompressor.class));
        when(mockFactory.getPigeon()).thenReturn(mock(IPigeonIMU.class));
        // for motors
        when(mockFactory.getMotor(anyString(), anyString()))
            .thenReturn(mock(IGreenMotor.class));
        when(
            mockFactory.getFollowerMotor(anyString(), anyString(), any(IGreenMotor.class))
        )
            .thenReturn(mock(IGreenMotor.class));
        // for subsystems
        var config = new SubsystemConfig();
        config.implemented = false;
        when(mockFactory.getSubsystem(anyString())).thenReturn(config);
        var slotConfig = new PIDSlotConfiguration();
        slotConfig.kD = 0d;
        slotConfig.kP = 0d;
        slotConfig.kI = 0d;
        slotConfig.kF = 0d;
        slotConfig.allowableError = 100d;
        slotConfig.iZone = 1;
        when(mockFactory.getPidSlotConfig(anyString())).thenReturn(slotConfig);
        var sm = mock(SwerveModule.class);
        when(sm.getActualState()).thenReturn(new SwerveModuleState());
        when(mockFactory.getSwerveModule(anyString(), anyString()))
            .thenAnswer(input -> sm);
        Injector.register(mockFactory);
    }
}
