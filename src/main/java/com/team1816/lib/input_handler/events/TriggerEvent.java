package com.team1816.lib.input_handler.events;

import com.team1816.lib.input_handler.controlOptions.Trigger;
import com.team1816.lib.util.team254.LatchedBoolean;

import java.util.ArrayList;

/**
 * An event that holds actions to run when the trigger's state changes.
 *
 * @see Trigger
 * @see Trigger.State
 */
public class TriggerEvent {
    private final Integer id;
    private final ArrayList<Runnable> pressActions = new ArrayList<>();
    private final ArrayList<Runnable> holdActions = new ArrayList<>();
    private final ArrayList<Runnable> releaseActions = new ArrayList<>();

    private final LatchedBoolean pressedState = new LatchedBoolean();
    private final LatchedBoolean releasedState = new LatchedBoolean();

    public TriggerEvent(Integer id) {
        this.id = id;
    }

    public Integer getId() {
        return this.id;
    }

    /**
     * Adds an action to be performed once when a trigger is pressed
     *
     * @param action
     */
    public void addPressAction(Runnable action) {
        pressActions.add(action);
    }

    /**
     * Adds an action to be performed while a trigger is held
     *
     * @param action
     */
    public void addHoldAction(Runnable action) {
        holdActions.add(action);
    }

    /**
     * Adds an action to be performed once when a trigger is released
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
