package com.team1816.season.auto.actions;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
import com.team1816.lib.util.team254.DriveSignal;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static com.team1816.lib.subsystems.Subsystem.factory;

/**
 * Action for infrastructure based / gyroscopic balancing
 * @see AutoAction
 */
public class AutoBalanceAction implements AutoAction {
    private static Drive drive;
    private static Infrastructure infrastructure;
    private static RobotState robotState;

    private static SwerveDriveKinematics swerveKinematics;
    private static DifferentialDriveKinematics tankKinematics;

    private static boolean isSwerve = false;
    private double maxVelocity;
    private double autoBalanceDivider = factory.getConstant(Drive.NAME, "autoBalanceDivider");

    public AutoBalanceAction(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }
    @Override
    public void start() {
        drive = Injector.get(Drive.Factory.class).getInstance();
        infrastructure = Injector.get(Infrastructure.class);
        robotState = Injector.get(RobotState.class);

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
        // splitting into x and y relative to robot components
        double pitch = -infrastructure.getPitch();
        double roll = infrastructure.getRoll();
        double velocityX = 0;
        double velocityY = 0;

        if(Math.abs(pitch) > 2 || Math.abs(roll) > 2){
            velocityX = pitch / autoBalanceDivider;
            velocityY = roll / autoBalanceDivider;
        }

        // I just realized that I'm a dingus and could use setTeleopInputs for this
        if (isSwerve) {
            drive.setTeleopInputs(velocityX, velocityY, 0);
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velocityX, velocityY, 0);
            DifferentialDriveWheelSpeeds wheelSpeeds = tankKinematics.toWheelSpeeds(chassisSpeeds);
            DriveSignal driveSignal = new DriveSignal(wheelSpeeds.leftMetersPerSecond/ TankDrive.kPathFollowingMaxVelMeters, wheelSpeeds.rightMetersPerSecond/TankDrive.kPathFollowingMaxVelMeters);
            ((TankDrive) drive).setVelocity(driveSignal);
        }
    }

    @Override
    public boolean isFinished() {
//        if (Math.abs(infrastructure.getPitch())<2 && Math.abs(infrastructure.getRoll())<2) {
//            return true;
//        }
        return false;
    }

    @Override
    public void done() {
        drive.setBraking(true);
    }
}
