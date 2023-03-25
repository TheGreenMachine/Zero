package com.team1816.lib.auto.modes;

import com.team1816.lib.DriveFactory;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.subsystems.drive.SwerveDrive;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;

/**
 * An abstract class that is the basis of a robot's autonomous routines.
 * Actions can be implemented in the routine and can be performed  (which are routines that do actions).
 */
public abstract class AutoMode {

    private static RobotState robotState;

    private static final long looperDtInMS = (long) (Constants.kLooperDt * 1000);

    /**
     * State: if mode needs to be stopped
     */
    private boolean needsStop;

    /**
     * State: Trajectory Actions to be run
     */
    protected List<TrajectoryAction> trajectoryActions;
    /**
     * State: Initial Pose that robot starts at
     */
    protected Pose2d initialPose;

    /**
     * Empty constructor for driveStraight and doNothing modes which don't require trajectories
     *
     * @see DoNothingMode
     * @see DriveStraightMode
     */
    protected AutoMode() {
        robotState = Injector.get(RobotState.class);

        if (robotState.allianceColor == Color.BLUE) {
            initialPose = new Pose2d(0,0, Rotation2d.fromDegrees(180));
        } else {
            initialPose = new Pose2d(0,0, Rotation2d.fromDegrees(0));
        }

    }

    /**
     * Instantiates an AutoMode from a list of trajectory actions
     *
     * @param trajectoryActions
     * @see TrajectoryAction
     */
    protected AutoMode(List<TrajectoryAction> trajectoryActions) {
        robotState = Injector.get(RobotState.class);


        this.trajectoryActions = trajectoryActions;
        boolean isSwerve = Injector.get(DriveFactory.class).getInstance() instanceof SwerveDrive;

        if (trajectoryActions.isEmpty()) {
            if (robotState.allianceColor == Color.BLUE) {
                initialPose = new Pose2d(0,0, Rotation2d.fromDegrees(180));
            } else {
                initialPose = new Pose2d(0,0, Rotation2d.fromDegrees(0));
            }
        } else {
            if (trajectoryActions.get(0).getTrajectoryHeadings() != null && isSwerve) {
                initialPose = new Pose2d(trajectoryActions.get(0).getTrajectory().getInitialPose().getTranslation(), trajectoryActions.get(0).getTrajectoryHeadings().get(0));
            } else {
                initialPose = trajectoryActions.get(0).getTrajectory().getInitialPose();
            }
        }
    }

    /**
     * Runs the autoMode routine actions
     *
     * @see #routine()
     */
    public void run() {
        start();

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE STOPPED EARLY ! ! !", false);
        }

        done();
    }

    /**
     * Starts the AutoMode and relevant actions
     */
    private void start() {
        System.out.println("Starting " + this.getClass().getName());
        needsStop = false;
    }

    /**
     * Routine register of actions that will be run in the mode
     *
     * @throws AutoModeEndedException
     */
    protected abstract void routine() throws AutoModeEndedException;

    /**
     * Standard cleanup end-procedure
     */
    protected void done() {
        System.out.println(this.getClass().getName() + " Done");
    }

    /**
     * Stops the auto mode
     */
    public void stop() {
        needsStop = true;
    }

    /**
     * Runs a given action, typically placed in routine()
     *
     * @param action
     * @throws AutoModeEndedException
     * @see AutoAction
     */
    protected void runAction(AutoAction action) throws AutoModeEndedException {
        action.start();

        // Run action, stop action on interrupt or done
        while (!action.isFinished()) {
            if (needsStop) {
                throw new AutoModeEndedException();
            }

            action.update();

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    /**
     * Gets current running Trajectory
     *
     * @return trajectory
     * @see Trajectory
     */
    public Trajectory getCurrentTrajectory() {
        if (trajectoryActions != null && trajectoryActions.size() > 0) {
            for (int i = 0; i < trajectoryActions.size(); i++) {
                if (!trajectoryActions.get(i).isFinished()) {
                    return trajectoryActions.get(i).getTrajectory();
                }
            }
        }
        return new Trajectory();
    }

    /**
     * Returns the initial pose of the robot
     *
     * @return initialPose
     */
    public Pose2d getInitialPose() {
        if (initialPose == null) {
            return Constants.kDefaultZeroingPose;
        }
        return initialPose;
    }

}
