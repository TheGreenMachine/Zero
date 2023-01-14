package com.team1816.lib.util.driveUtil;

import com.team1816.lib.util.team254.DriveSignal;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface DriveKinematics {
    DriveSignal inverseKinematics(ChassisSpeeds chassisSpeeds);
}
