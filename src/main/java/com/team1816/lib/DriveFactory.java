package com.team1816.lib;

import com.google.inject.Singleton;
import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;

/**
 * Decides between drivetrain type based on factory constants. Allows for differential / swerve duality.
 *
 * @see Drive
 */
@Singleton
public class DriveFactory implements Drive.Factory {

    private static Drive mDrive;

    @Override
    public Drive getInstance() {
        if (mDrive == null) {
            boolean isSwerve =
                Injector.get(RobotFactory.class).getConstant(Drive.NAME, "isSwerve") == 1;
            if (isSwerve) {
                mDrive = Injector.get(SwerveDrive.class);
            } else {
                mDrive = Injector.get(TankDrive.class);
            }
            System.out.println("Created " + mDrive.getClass().getSimpleName());
        }
        return mDrive;
    }
}
