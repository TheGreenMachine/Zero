package com.team1816.season.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.turret.Turret;

public class TurretAction implements AutoAction {

    private static Turret turret;

    private double turretAngle;

    public TurretAction(double turretAngle) {
        this.turretAngle = turretAngle;
        turret = Injector.get(Turret.class);
    }

    @Override
    public void start() {
        turret.setTurretAngle(turretAngle);
        System.out.println("setting turret angle to " + turretAngle);
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
