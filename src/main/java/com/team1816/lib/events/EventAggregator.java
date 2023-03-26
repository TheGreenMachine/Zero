package com.team1816.lib.events;

import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.HashMap;

/**
 * A class the aggregates events into a registry of events that can then be called upon to trigger an event
 *
 * @see IEventAggregator
 */
public class EventAggregator implements IEventAggregator {

    private final HashMap<Object, EventBase> _events = new HashMap<>();

    @Override
    public <TEventType extends EventBase> TEventType GetEvent(Class<TEventType> type) {
        if (_events.containsKey(type)) return type.cast(_events.get(type));
        TEventType newEvent;
        try {
            newEvent = type.getDeclaredConstructor().newInstance();
        } catch (Exception exp) {
            GreenLogger.log(exp.getMessage());
            return null;
        }
        _events.put(type, newEvent);
        return newEvent;
    }
}
