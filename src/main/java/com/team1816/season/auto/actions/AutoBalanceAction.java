package com.team1816.season.auto.actions;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.season.Robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * Action for infrastructure based / gyroscopic balancing
 *
 * @see AutoAction
 */
public class AutoBalanceAction implements AutoAction {
    private static Drive drive;
    private static Infrastructure infrastructure;
    private static SwerveDriveKinematics swerveKinematics;
    private static DifferentialDriveKinematics tankKinematics;

    private static boolean isSwerve = false;
    private double maxVelocity;

    public AutoBalanceAction(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    @Override
    public void start() {
        drive = Injector.get(Drive.Factory.class).getInstance();
        infrastructure = Injector.get(Infrastructure.class);

        if (drive instanceof SwerveDrive) {
            isSwerve = true;
        }
        if (isSwerve) {
            swerveKinematics = ((SwerveDrive) drive).getKinematics();
        } else {
            tankKinematics = ((TankDrive) drive).getKinematics();
        }
    }

    @Override
    public void update() { // TODO Implement proximity
        double pitch = -infrastructure.getPitch();
        double roll = infrastructure.getRoll();
        double velocityX = 0;
        double velocityY = 0;

        if(Math.abs(pitch) > 1 || Math.abs(roll) > 1) {
            velocityX = pitch / 40;
            velocityY = roll / 40;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velocityX, velocityY, 0);

        if (isSwerve) {
            ((SwerveDrive) drive).setModuleStates(swerveKinematics.toSwerveModuleStates(chassisSpeeds));
        } else {
            DifferentialDriveWheelSpeeds wheelSpeeds = tankKinematics.toWheelSpeeds(chassisSpeeds);
            DriveSignal driveSignal = new DriveSignal(wheelSpeeds.leftMetersPerSecond / TankDrive.kPathFollowingMaxVelMeters, wheelSpeeds.rightMetersPerSecond / TankDrive.kPathFollowingMaxVelMeters);
            ((TankDrive) drive).setVelocity(driveSignal);
        }
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - Robot.teleopStart) > 134 || (Math.abs(infrastructure.getPitch()) < 2 && Math.abs(infrastructure.getRoll()) < 2);
    }

    @Override
    public void done() {
        drive.setBraking(true);
    }
}
