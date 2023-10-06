package com.team1816.lib.auto.actions;

import com.team1816.lib.Injector;
import com.team1816.lib.loops.AsyncTimer;
import com.team1816.lib.subsystems.drive.Drive;

public class DriveOpenLoopAction implements AutoAction {

    private static Drive drive;

    private final AsyncTimer driveTimer;

    public DriveOpenLoopAction(double driveTime, double percentOutput) {
        drive = Injector.get(Drive.Factory.class).getInstance();
        this.driveTimer =
            new AsyncTimer(
                driveTime,
                () -> drive.setTeleopInputs(percentOutput, 0, 0),
                () -> drive.setTeleopInputs(0, 0, 0)
            );
    }

    public DriveOpenLoopAction(double driveTime, double percentOutput, double rotation) {
        drive = Injector.get(Drive.Factory.class).getInstance();
        this.driveTimer =
            new AsyncTimer(
                driveTime,
                () -> drive.setTeleopInputs(percentOutput, 0, 0),
                () -> drive.setTeleopInputs(percentOutput, 0, rotation),
                () -> drive.setTeleopInputs(0, 0, 0)
            );
    }

    @Override
    public void start() {
        driveTimer.update();
    }

    @Override
    public void update() {
        driveTimer.update();
    }

    @Override
    public boolean isFinished() {
        return driveTimer.isCompleted();
    }

    @Override
    public void done() {
        drive.setTeleopInputs(0, 0, 0);
    }
}
