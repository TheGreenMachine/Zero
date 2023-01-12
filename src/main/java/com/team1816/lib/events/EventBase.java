package com.team1816.lib.events;

import java.util.ArrayList;

/**
 * The base event that all events extend from
 *
 * @see PubSubRunnable
 * @see PubSubConsumer
 */
public abstract class EventBase {

    /**
     * Event subscriptions
     */
    protected ArrayList<IEventSubscription> Subscriptions = new ArrayList<>();

    /**
     * Adds subscription
     *
     * @param subscription to be invoked when {@link #InternalPublish(Object)} is called
     */
    protected void InternalSubscribe(IEventSubscription subscription) {
        Subscriptions.add(subscription);
    }

    /**
     * Publishes date to subscribers
     *
     * @param parameter the value to pass to the {@link #Subscriptions}
     */
    protected void InternalPublish(Object parameter) {
        for (var sub : Subscriptions) {
            if (sub.IsRunnable()) {
                sub.GetRunnable().run();
            } else {
                sub.GetConsumer().accept(parameter);
            }
        }
    }
}
