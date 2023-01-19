package com.team1816.season.subsystems;

import com.team1816.TestUtil;
import com.team1816.lib.Injector;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
import junit.framework.TestCase;
import org.junit.Before;

public class OrchestratorTest extends TestCase {

    private final RobotState state;

    private Orchestrator mOrchestrator;

    public OrchestratorTest() {
        TestUtil.SetupMockRobotFactory(null);
        state = Injector.get(RobotState.class);
    }

    @Before
    public void setUp() {
        mOrchestrator = Injector.get(Orchestrator.class);
        state.resetPosition();
    }

    public void testSetStopped() {
    }

    public void testSetCollecting() {
    }

    public void testSetRevving() {
    }

    public void testSetFiring() {
    }

    public void testGetDistance() {
    }

    public void testGetPredictedDistance() {
    }
}
