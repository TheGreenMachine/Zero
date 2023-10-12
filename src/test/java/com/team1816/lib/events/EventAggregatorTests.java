package com.team1816.lib.events;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class EventAggregatorTests {

    private String _eventText = "";

    protected static class StringEvent extends PubSubConsumer<String> {
    }

    protected static class StringEvent2 extends PubSubConsumer<String> {
    }

    protected static class RunnableEvent extends PubSubRunnable {
    }

    private EventAggregator _target;

    @BeforeEach
    public void TestInit() {
        _target = new EventAggregator();
        _eventText = "";
    }

    @Test
    public void GetEventTest() {
        var evt = _target.GetEvent(StringEvent.class);
        // verify same class gets returned
        Assertions.assertEquals(
            evt.hashCode(),
            _target.GetEvent(StringEvent.class).hashCode(),
            "New class created"
            );
        evt.Subscribe(this::EventFired);
        evt.Publish("foobar");
        Assertions.assertEquals("foobar", _eventText);
    }

    @Test
    public void GetEventTestNoData() {
        var evt = _target.GetEvent(RunnableEvent.class);
        evt.Subscribe(
            () -> {
                _eventText = "noData";
            }
        );
        evt.Publish();
        Assertions.assertEquals("noData", _eventText);
    }

    @Test
    public void GetEventTestSingle() {
        //Ensure that event system only fires on the published event
        var evt2 = _target.GetEvent(StringEvent2.class);
        evt2.Subscribe(this::EventFired);
        var evt = _target.GetEvent(StringEvent.class);
        evt.Subscribe(this::EventFired);
        evt.Publish("single");
        Assertions.assertEquals("single", _eventText);
    }

    private void EventFired(String arg) {
        _eventText += arg;
    }
}
