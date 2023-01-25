package com.team1816.lib.events;

import java.util.function.Consumer;

/**
 * Base class that all events that send data extend from
 *
 * @param <T> T is the type of data that is sent
 * @see EventBase
 */
public class PubSubConsumer<T> extends EventBase {

    /**
     * @param action this is the consumer ( the method that listens to the event)
     */
    public void Subscribe(Consumer<T> action) {
        InternalSubscribe(new EventSubscription<>(action));
    }

    /**
     * Sends an event to the registered consumers
     *
     * @param parameter this is the data to publish to the subscribers
     */
    public void Publish(T parameter) {
        InternalPublish(parameter);
    }
}
