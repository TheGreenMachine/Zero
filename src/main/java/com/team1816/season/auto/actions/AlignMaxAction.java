package com.team1816.season.auto.actions;

import com.team1816.season.subsystems.Elevator;

public class AlignMaxAction extends AlignAction {
    public AlignMaxAction() {
        super(Elevator.EXTENSION_STATE.MAX, (Elevator.EXTENSION_STATE.MID.getExtension() + Elevator.EXTENSION_STATE.MAX.getExtension()) / 2d, Elevator.EXTENSION_STATE.MAX.getExtension());
    }
}
