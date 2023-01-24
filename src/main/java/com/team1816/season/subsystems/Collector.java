package com.team1816.season.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.Infrastructure;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.pcm.ISolenoid;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.states.RobotState;
// remember to add @Singleton here! - Injector will make more than once instance of this class otherwise
@Singleton
public class Collector extends Subsystem {

    private final ISolenoid collectorPiston;

    private final IGreenMotor intakeMotor;

    private double intakeVel;

    private PIVOT_STATE desiredPivotState = PIVOT_STATE.DOWN;

    private COLLECTOR_STATE desiredCollectorState = COLLECTOR_STATE.STOP;

    private boolean outputsChanged = false;

    private static final String NAME = "collector";

    // Remember to add @Inject :)
    @Inject
    public Collector (Infrastructure inf, RobotState rs) {
        super(NAME, inf, rs);
        // I'd keep both yml and subsystem names the same to avoid mess-ups
        collectorPiston = factory.getSolenoid(NAME, "collectorSolenoid");
        intakeMotor = factory.getMotor(NAME, "intakeMotor");
        /*
        just as a precaution, since we're going to be stalling the intake motor throughout the match,
        we need to configure the motor's current limit to prevent motor dmg - for the NEO 550 that we're using,
        20 amps is the recommended limit
        */
        intakeMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 20, 0,0),
            0
        );
    }

    public void setDesiredState(PIVOT_STATE pivotState, COLLECTOR_STATE collectorState){
        if (desiredPivotState != pivotState) {
            desiredPivotState = pivotState;
            outputsChanged = true;
        }
        if (desiredCollectorState != collectorState) {
            desiredCollectorState = collectorState;
            outputsChanged = true;
        }
    }
    @Override
    public void readFromHardware() {
        // we'll have to add some logic in here to check if the motor is or isn't stalling to update the collector's actual state
        intakeVel = intakeMotor.getSelectedSensorVelocity(0);
    }

    @Override
    public void writeToHardware() {
        if (outputsChanged) {
            outputsChanged = false;
            switch (desiredPivotState) {
                case UP -> collectorPiston.set(true);
                case DOWN -> collectorPiston.set(false);
            }
            switch (desiredCollectorState) {
                case STOP -> intakeMotor.set(ControlMode.Velocity, 0);
                // better to make a variable in collector called collectVel and flushVel that u set to what u have in yml
                // than repeatedly call into factory to get the value - calling stuff from the factory uses more processing power
                case COLLECT -> intakeMotor.set(ControlMode.Velocity, factory.getConstant(NAME, "collectTicks"));
                case FLUSH -> intakeMotor.set(ControlMode.Velocity, factory.getConstant(NAME, "flushTicks"));
            }
        }
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean testSubsystem() {
        return false;
    }

    // for cube collecting, the "motor locked" state, so we don't pop the cube will have
    // to be the same as stopping? a hold action and then when it's not going it stops? we can't
    // rotate after getting a cube anyway

    // I think (if we're running it in %out) we should just keep it in collect mode - stop mode will tell our motors to go at 0% power,
    // which will cause them to go slack and let the cube fall out - this is where the intentional motor stalling will come in -
    public enum PIVOT_STATE {
        UP,
        DOWN,
    }

    public enum COLLECTOR_STATE {
        STOP,
        COLLECT,
        FLUSH,
    }
}
