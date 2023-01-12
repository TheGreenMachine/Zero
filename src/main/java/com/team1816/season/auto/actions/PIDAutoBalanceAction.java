package com.team1816.season.auto.actions;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.subsystems.drive.Drive;

public class PIDAutoBalanceAction implements AutoAction {

    private Infrastructure infrastructure;
    private static Drive drive;

    public PIDAutoBalanceAction(){
        infrastructure = Injector.get(Infrastructure.class);
        drive = Injector.get(Drive.Factory.class).getInstance();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        double theta = infrastructure.getPitch();
        if(Math.abs(theta) > 1)
            drive.setTeleopInputs((-theta)/308,0,0);

    }

    @Override
    public boolean isFinished() {
        double theta = infrastructure.getPitch();
        if(theta <= 1 && theta >= -1)
            return true;
        return false;
    }

    @Override
    public void done() {
        drive.setBraking(true);
    }
}
