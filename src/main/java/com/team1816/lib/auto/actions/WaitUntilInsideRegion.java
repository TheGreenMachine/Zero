package com.team1816.lib.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.Symmetry;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.configuration.Constants;
import com.team1816.season.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

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
    private Translation2d mBottomLeft;
    /**
     * State: Top right of rectangular region to wait inside
     */
    private Translation2d mTopRight;
    /**
     * State: Name of region
     */
    private String name = "";

    private Color allianceColor;

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
        allianceColor = Color.BLUE;
    }

    public WaitUntilInsideRegion(
        Translation2d bottomLeft,
        Translation2d topRight,
        String name,
        Color color
    ) {
        this.name = name;
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
        mRobotState = Injector.get(RobotState.class);
        allianceColor = color;
        if (Constants.fieldSymmetry == Symmetry.AXIS && color == Color.RED) {
            mBottomLeft = new Translation2d(Constants.fieldCenterX * 2 - mTopRight.getX() - Math.abs(topRight.getX() - bottomLeft.getX()), mBottomLeft.getY());
            mTopRight = new Translation2d(Constants.fieldCenterX * 2 - mBottomLeft.getX() + Math.abs(topRight.getX() - bottomLeft.getX()), mTopRight.getY());
        }
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
        var x = (position.getX());
        var y = (position.getY());
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
        GreenLogger.log("INSIDE DESIRED REGION: " + name);
    }
}
