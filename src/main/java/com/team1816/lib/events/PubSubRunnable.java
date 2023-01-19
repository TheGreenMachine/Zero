package com.team1816.lib.events;

/**
 * PubSub event that does not pass any data
 *
 * @see EventBase
 */
public class PubSubRunnable extends EventBase {

    public void Subscribe(Runnable action) {
        InternalSubscribe(new EventSubscription<>(action));
    }

    public void Publish() {
        InternalPublish(null);
    }
}
