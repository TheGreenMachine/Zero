package com.team1816.season.auto.actions;

import com.team1816.season.subsystems.Elevator;

public class AlignMinAction extends AlignAction {
    public AlignMinAction() {
        super(Elevator.EXTENSION_STATE.MIN, 0, (Elevator.EXTENSION_STATE.MIN.getExtension() + Elevator.EXTENSION_STATE.MID.getExtension()) / 2d);
    }
}
