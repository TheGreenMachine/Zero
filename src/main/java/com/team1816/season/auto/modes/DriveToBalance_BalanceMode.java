package com.team1816.season.auto.modes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.subsystems.turret.Turret;
import com.team1816.season.auto.actions.AutoBalanceAction;
import com.team1816.season.auto.actions.CollectAction;
import com.team1816.season.auto.actions.ElevatorAction;
import com.team1816.season.auto.paths.DriveToBalance_Balance;
import com.team1816.season.auto.paths.LivingRoomPath;
import com.team1816.season.subsystems.Collector;
import com.team1816.season.subsystems.Elevator;
import org.checkerframework.checker.units.qual.C;

import java.util.List;

public class DriveToBalance_BalanceMode extends AutoMode {

    public DriveToBalance_BalanceMode(){
        super(List.of(new TrajectoryAction(new DriveToBalance_Balance())));
    }

    public DriveToBalance_BalanceMode(Color color) {
        super(List.of(new TrajectoryAction(new DriveToBalance_Balance(color))));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running DriveToBalance_BalanceMode AutoMode");
        runAction(
            new SeriesAction(
                new WaitAction(.5),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(2),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE_DIP, Elevator.EXTENSION_STATE.MAX),
                new WaitAction(.25),
                new CollectAction(ControlMode.Velocity, Collector.PIVOT_STATE.UP, Collector.ROLLER_STATE.OUTTAKE),
                new WaitAction(.5),
                new CollectAction(ControlMode.Velocity, Collector.PIVOT_STATE.UP, Collector.ROLLER_STATE.STOP),
                new ElevatorAction(Elevator.ANGLE_STATE.SCORE, Elevator.EXTENSION_STATE.MIN),
                new WaitAction(.5),
                trajectoryActions.get(0),
                new AutoBalanceAction()
            )
        );

    }
}
