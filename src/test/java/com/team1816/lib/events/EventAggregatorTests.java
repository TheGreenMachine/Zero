package com.team1816.lib.events;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

public class EventAggregatorTests {

    private String _eventText = "";

    protected static class StringEvent extends PubSubConsumer<String> {}

    protected static class StringEvent2 extends PubSubConsumer<String> {}

    protected static class RunnableEvent extends PubSubRunnable {}

    private EventAggregator _target;

    @Before
    public void TestInit() {
        _target = new EventAggregator();
        _eventText = "";
    }

    @Test
    public void GetEventTest() {
        var evt = _target.GetEvent(StringEvent.class);
        // verify same class gets returned
        Assert.assertEquals(
            "New class created",
            evt.hashCode(),
            _target.GetEvent(StringEvent.class).hashCode()
        );
        evt.Subscribe(this::EventFired);
        evt.Publish("foobar");
        Assert.assertEquals("foobar", _eventText);
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
        Assert.assertEquals("noData", _eventText);
    }

    @Test
    public void GetEventTestSingle() {
        //Ensure that event system only fires on the published event
        var evt2 = _target.GetEvent(StringEvent2.class);
        evt2.Subscribe(this::EventFired);
        var evt = _target.GetEvent(StringEvent.class);
        evt.Subscribe(this::EventFired);
        evt.Publish("single");
        Assert.assertEquals("single", _eventText);
    }

    private void EventFired(String arg) {
        _eventText += arg;
    }
}
