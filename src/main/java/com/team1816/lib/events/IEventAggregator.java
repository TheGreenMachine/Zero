package com.team1816.lib.events;

import java.lang.reflect.InvocationTargetException;

/**
 * Interface for event aggregator
 */
public interface IEventAggregator {
    <TEventType extends EventBase> TEventType GetEvent(Class<TEventType> type)
        throws NoSuchMethodException, InvocationTargetException, InstantiationException, IllegalAccessException;
}
