package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.turret.Turret;

public class AbsoluteTurretAction implements AutoAction {

    private static Turret turret;

    public AbsoluteTurretAction() {
        turret = Injector.get(Turret.class);
    }

    @Override
    public void start() {
        turret.setControlMode(Turret.ControlMode.ABSOLUTE_FOLLOWING);
        System.out.println("setting turret mode to absolute");
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}
