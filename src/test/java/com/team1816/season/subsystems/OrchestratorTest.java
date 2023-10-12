package com.team1816.season.subsystems;

import com.team1816.TestUtil;
import com.team1816.lib.Injector;
import com.team1816.season.states.Orchestrator;
import com.team1816.season.states.RobotState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


public class OrchestratorTest {
    private final RobotState state;

    private Orchestrator mOrchestrator;

    public OrchestratorTest() {
        TestUtil.SetupMockRobotFactory(null);
        state = Injector.get(RobotState.class);
    }

    @BeforeEach
    public void setUp() {
        mOrchestrator = Injector.get(Orchestrator.class);
        state.resetPosition();
    }

    @Test
    public void testSetStopped() {
    }

    @Test
    public void testSetCollecting() {
    }

    @Test
    public void testSetRevving() {
    }

    @Test
    public void testSetFiring() {
    }

    @Test
    public void testGetDistance() {
    }

    @Test
    public void testGetPredictedDistance() {
    }
}
