package com.team1816.lib.events;

import java.util.function.Consumer;

/**
 * Interface for event subscriptions
 */
public interface IEventSubscription {
    <T> Consumer<T> GetConsumer();
    Runnable GetRunnable();
    boolean IsRunnable();
}
