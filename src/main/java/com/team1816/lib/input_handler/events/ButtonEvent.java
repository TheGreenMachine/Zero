package com.team1816.lib.input_handler.events;

import com.team1816.lib.input_handler.controlOptions.Button;
import com.team1816.lib.util.team254.LatchedBoolean;

import java.util.ArrayList;

/**
 * An event that holds actions to run when the button's state changes.
 *
 * @see Button
 * @see Button.State
 */
public class ButtonEvent {
    private final Integer id;
    private final ArrayList<Runnable> pressActions = new ArrayList<>();
    private final ArrayList<Runnable> holdActions = new ArrayList<>();
    private final ArrayList<Runnable> releaseActions = new ArrayList<>();

    private final LatchedBoolean pressedState = new LatchedBoolean();
    private final LatchedBoolean releasedState = new LatchedBoolean();

    public ButtonEvent(Integer id) {
        this.id = id;
    }

    public Integer getId() {
        return this.id;
    }

    /**
     * Adds an action to be performed once when a button is pressed
     *
     * @param action
     */
    public void addPressAction(Runnable action) {
        pressActions.add(action);
    }

    /**
     * Adds an action to be performed while a button is held
     *
     * @param action
     */
    public void addHoldAction(Runnable action) {
        holdActions.add(action);
    }

    /**
     * Adds an action to be performed once when a button is released
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
