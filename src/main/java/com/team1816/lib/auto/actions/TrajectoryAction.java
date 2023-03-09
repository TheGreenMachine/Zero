package com.team1816.lib.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.lib.subsystems.drive.TankDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import static com.team1816.lib.subsystems.drive.Drive.*;
import static com.team1816.lib.subsystems.drive.SwerveDrive.swerveKinematics;

/**
 * This class represents a runnable action that will allow a drivetrain to follow a trajectory.
 *
 * @see AutoAction
 */
public class TrajectoryAction implements AutoAction {

    /**
     * Command for drivetrain
     *
     * @see Command
     */
    private final Command command;

    /**
     * Trajectory (list of states) for drivetrain to follow
     *
     * @see Trajectory
     */
    private final Trajectory trajectory;

    /**
     * List of headings for swerve commands
     */
    private final List<Rotation2d> headings;

    /**
     * Drivetrain (tank or swerve)
     *
     * @see Drive
     * @see TankDrive
     * @see SwerveDrive
     */
    private final Drive drive;

    /**
     * Constructs a TrajectoryAction based on an AutoPath which contains a trajectory and heading
     *
     * @param autoPath
     * @see AutoPath
     */
    public TrajectoryAction(AutoPath autoPath) {
        this(autoPath.getAsTrajectory(), autoPath.getAsTrajectoryHeadings());
    }

    /**
     * Main constructor of a TrajectoryAction, instantiates and assigns the command based on the drivetrain
     *
     * @param trajectory
     * @param headings
     * @see Trajectory
     * @see Drive
     * @see Command
     */
    public TrajectoryAction(Trajectory trajectory, List<Rotation2d> headings) {
        drive = Injector.get(Drive.Factory.class).getInstance();
        this.trajectory = trajectory;
        this.headings = headings;

        // create command (wpi version of an action)
        if (drive instanceof TankDrive) {
            command =
                new RamseteCommand(
                    trajectory,
                    drive::getPose,
                    new RamseteController(), //defaults of
                    new DifferentialDriveKinematics(
                        kDriveWheelTrackWidthMeters
                    ),
                    ((TankDrive) drive)::updateTrajectoryVelocities
                );
        } else if (drive instanceof SwerveDrive) {
            var thetaController = new ProfiledPIDController(
                kPThetaController,
                0,
                0,
                kThetaControllerConstraints
            );
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            command =
                new SwerveControllerCommand(
                    trajectory,
                    drive::getPose,
                    swerveKinematics,
                    new PIDController(kPXController, 0, kDXController),
                    new PIDController(kPYController, 0, kDYController),
                    thetaController,
                    ((SwerveDrive) drive)::getTrajectoryHeadings,
                    ((SwerveDrive) drive)::setModuleStates
                );
        } else {
            System.out.println(
                " oh man oh god I'm neither swerve nor tank! " + drive.toString()
            );
            command = null;
        }
    }

    /**
     * Returns the trajectory that is associated with the action
     *
     * @return trajectory
     * @see Trajectory
     */
    public Trajectory getTrajectory() {
        return trajectory;
    }

    /**
     * Returns the trajectory headings that are associated with the action
     *
     * @return trajectory headings
     */
    public List<Rotation2d> getTrajectoryHeadings() {
        return headings;
    }

    /**
     * Starts the command, executes trajectory on drivetrain
     *
     * @see Drive#startTrajectory(Trajectory, List)
     * @see Command
     * @see AutoAction#start()
     */
    @Override
    public void start() {
        System.out.println(
            "Starting trajectory! (seconds = " + trajectory.getTotalTimeSeconds() + ")"
        );
        drive.startTrajectory(trajectory, headings);
        command.initialize();
    }

    /**
     * Executes the command
     *
     * @see Command
     * @see Command#execute()
     * @see AutoAction#update()
     */
    @Override
    public void update() {
        command.execute();
    }

    /**
     * Returns whether or not the command has been executed
     *
     * @return boolean isFinished
     * @see Command
     * @see AutoAction#isFinished()
     */
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    /**
     * Ends the command, stops drivetrain
     *
     * @see Drive
     * @see Command
     * @see AutoAction#done()
     */
    @Override
    public void done() {
        command.end(false);
        drive.stop();
    }
}
