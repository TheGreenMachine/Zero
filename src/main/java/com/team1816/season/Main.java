/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1816.season;

import com.team1816.lib.auto.newrevisedpathingstuff.BetterTrajectoryPathing;
import com.team1816.lib.auto.paths.PathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.List;
import java.util.Scanner;

import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxAccelMeters;
import static com.team1816.lib.subsystems.drive.Drive.kPathFollowingMaxVelMeters;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what you are doing, do not modify this file except to
 * change the parameter class to the startRobot call.
 */
public final class Main {

    private Main() {
    }

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
//        Scanner sc = new Scanner(System.in);
//
//        Trajectory traj = PathUtil.generateTrajectory(false,
//                List.of(
//                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
//                        new Pose2d(5.43, 0.86, Rotation2d.fromDegrees(0)),
//                        new Pose2d(8.7, 4.02, Rotation2d.fromDegrees(0)),
//                        new Pose2d(5.14, 5.28, Rotation2d.fromDegrees(0)),
//                        new Pose2d(9.27, 3.99, Rotation2d.fromDegrees(0))
//                )
//        );
//
//        System.out.println(traj);
//
//        double x = 1;
//        double y = 0;
//        for(int i = 0; i < 100; i++){
//            y+=1;
//            y-=0.99;
//        }
//        System.out.println(x);
//        System.out.println(y);
//        System.out.println(x == y);
//
//        Trajectory traj2 = BetterTrajectoryPathing.calculateTrajectory(
//                null,
//                new TrajectoryConfig(kPathFollowingMaxVelMeters, kPathFollowingMaxAccelMeters),
//                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
//                new Pose2d(10, 10, Rotation2d.fromDegrees(0)),
//                false,
//                true
//        );
//
//        System.out.println(traj2);
//        System.out.println(traj2.getTotalTimeSeconds());
//        System.out.println(traj2.getStates());

        RobotBase.startRobot(Robot::new);
    }
}