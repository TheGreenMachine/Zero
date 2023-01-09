package com.team1816.season.auto.actions;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.Action;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
import com.team1816.lib.util.team254.DriveSignal;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class AutoBalanceAction implements Action {
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
    public void update() {
        double velocity = 0;
        if (Math.abs(infrastructure.getPitch())>2) {
            velocity = (1 - Math.cos(Units.degreesToRadians(45d / 11 * infrastructure.getPitch())))*maxVelocity;
        }
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velocity, 0, 0);
        if (isSwerve) {
            ((SwerveDrive) drive).setModuleStates(swerveKinematics.toSwerveModuleStates(chassisSpeeds));
        } else {
            DifferentialDriveWheelSpeeds wheelSpeeds = tankKinematics.toWheelSpeeds(chassisSpeeds);
            DriveSignal driveSignal = new DriveSignal(wheelSpeeds.leftMetersPerSecond/TankDrive.kPathFollowingMaxVelMeters, wheelSpeeds.rightMetersPerSecond/TankDrive.kPathFollowingMaxVelMeters);
            ((TankDrive) drive).setVelocity(driveSignal);
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(infrastructure.getPitch())<2 && Math.abs(infrastructure.getRoll())<2) {
            return true;
        }
        return false;
    }

    @Override
    public void done() {
        drive.setBraking(true);
    }
}
