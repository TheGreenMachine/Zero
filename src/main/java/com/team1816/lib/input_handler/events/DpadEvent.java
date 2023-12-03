package com.team1816.lib.input_handler.events;

import com.team1816.lib.input_handler.controlOptions.Dpad;
import com.team1816.lib.util.team254.LatchedBoolean;

import java.util.ArrayList;

/**
 * An event that holds actions to run when the Dpad's state changes.
 *
 * @see Dpad
 * @see Dpad.State
 */
public class DpadEvent {
    private final ArrayList<Runnable> pressActions = new ArrayList<>();
    private final ArrayList<Runnable> holdActions = new ArrayList<>();
    private final ArrayList<Runnable> releaseActions = new ArrayList<>();

    private final LatchedBoolean pressedState = new LatchedBoolean();
    private final LatchedBoolean releasedState = new LatchedBoolean();

    public DpadEvent() {}

    /**
     * Adds an action to be performed once when a dpad axis is pressed
     *
     * @param action
     */
    public void addPressAction(Runnable action) {
        pressActions.add(action);
    }

    /**
     * Adds an action to be performed while a dpad axis is pressed
     *
     * @param action
     */
    public void addHoldAction(Runnable action) {
        holdActions.add(action);
    }

    /**
     * Adds an action to be performed once when a dpad axis is released
     *
     * @param action
     */
    public void addReleaseAction(Runnable action) {
        releaseActions.add(action);
    }

    public void publish(boolean held) {
        boolean justPressed = pressedState.update(held);
        boolean justReleased = releasedState.update(!held);

        if (justPressed) {
            for (var action : pressActions) {
                action.run();
            }
        }

        if (held) {
            for (var action : holdActions) {
                action.run();
            }
        }

        if (justReleased) {
            for (var action : releaseActions) {
                action.run();
            }
        }
    }
}
