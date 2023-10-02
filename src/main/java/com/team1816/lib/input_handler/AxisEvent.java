package com.team1816.lib.input_handler;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * An event that holds actions to run when the axes' states change.
 *
 * @see Axis
 */
public class AxisEvent {
    private final Integer id;
    private final ArrayList<Consumer<Double>> actions = new ArrayList<>();

    public AxisEvent(Integer id) {
        this.id = id;
    }

    public Integer getId() {
        return this.id;
    }

    public void addAction(Consumer<Double> action) {
        actions.add(action);
    }

    public void publish(Double value) {
        for (var action : actions) {
            action.accept(value);
        }
    }
}
