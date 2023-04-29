package com.team1816.season.auto.actions;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;

/**
 * Action for infrastructure based / gyroscopic balancing
 *
 * @see AutoAction
 */
public class AutoBalanceAction implements AutoAction {
    private static LedManager ledManager;
    private static Drive drive;
    private static Infrastructure infrastructure;
    private static RobotState robotState;

    private static SwerveDriveKinematics swerveKinematics;
    private static DifferentialDriveKinematics tankKinematics;

    private static boolean isSwerve = false;

    public AutoBalanceAction() {
    }

    @Override
    public void start() {
        ledManager = Injector.get(LedManager.class);
        drive = Injector.get(Drive.Factory.class).getInstance();
        infrastructure = Injector.get(Infrastructure.class);
        robotState = Injector.get(RobotState.class);

        isSwerve = drive instanceof SwerveDrive;

        GreenLogger.log("Initiating balance procedure!");
        ledManager.indicateStatus(LedManager.RobotStatus.BALANCE, LedManager.ControlState.BLINK);
        drive.autoBalance(new ChassisSpeeds());
    }

    @Override
    public void update() {
        drive.autoBalance(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - Robot.autoStart) > 14.5;
    }

    @Override
    public void done() {
        GreenLogger.log("Drivetrain is balanced!");
        ledManager.indicateStatus(LedManager.RobotStatus.BALANCE, LedManager.ControlState.SOLID);
        drive.setBraking(true);
    }
}
