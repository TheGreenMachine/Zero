package com.team1816.lib.events;

import java.util.function.Consumer;

/**
 * Implementation of an event that sends data sed to store the Consumer (action) to be called
 * @param <T> this is the type of data that publish will fire
 */
public class EventSubscription<T> implements IEventSubscription {

    private Consumer<T> _consumerAction = null;
    private Runnable _runnableAction = null;

    public EventSubscription(Consumer<T> action) {
        _consumerAction = action;
    }

    public EventSubscription(Runnable action) {
        _runnableAction = action;
    }

    @Override
    public Consumer<T> GetConsumer() {
        return _consumerAction;
    }

    @Override
    public Runnable GetRunnable() {
        return _runnableAction;
    }

    public boolean IsRunnable() {
        return _consumerAction == null;
    }
}
