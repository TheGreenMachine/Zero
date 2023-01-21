package com.team1816.lib.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Action that stops processes / waits inside a region.
 * Dependents: RobotState
 *
 * @see RobotState
 * @see AutoAction
 */
public class WaitUntilInsideRegion implements AutoAction {

    /**
     * State: RobotState
     *
     * @see RobotState
     */
    private static RobotState mRobotState;

    /**
     * State: Bottom left of rectangular region to wait inside
     */
    private final Translation2d mBottomLeft;
    /**
     * State: Top right of rectangular region to wait inside
     */
    private final Translation2d mTopRight;
    /**
     * State: Name of region
     */
    private String name = "";

    /**
     * Instantiates a WaitUntilInsideRegion action based on state parameters
     *
     * @param bottomLeft
     * @param topRight
     * @param name
     */
    public WaitUntilInsideRegion(
        Translation2d bottomLeft,
        Translation2d topRight,
        String name
    ) {
        this.name = name;
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
        mRobotState = Injector.get(RobotState.class);
    }

    /**
     * Starts the action (empty)
     *
     * @see AutoAction#start()
     */
    @Override
    public void start() {
    }

    /**
     * Updates the action (empty)
     *
     * @see AutoAction#update()
     */
    @Override
    public void update() {
    }

    /**
     * Checks if position criteria is met based on robotState
     *
     * @return boolean isFinished
     * @see AutoAction#isFinished()
     */
    @Override
    public boolean isFinished() {
        Pose2d position = mRobotState.fieldToVehicle;
        var x = Units.metersToInches(position.getX());
        var y = Units.metersToInches(position.getY());
        return (
            x > mBottomLeft.getX() &&
                x < mTopRight.getX() &&
                y > mBottomLeft.getY() &&
                y < mTopRight.getY()
        );
    }

    /**
     * Standard cleanup procedure: prints out action
     *
     * @see AutoAction#done()
     */
    @Override
    public void done() {
        System.out.println("INSIDE DESIRED REGION: " + name);
    }
}
