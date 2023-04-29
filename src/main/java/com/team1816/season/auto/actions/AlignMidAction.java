package com.team1816.season.auto.actions;

import com.team1816.season.subsystems.Elevator;

public class AlignMidAction extends AlignAction {
    public AlignMidAction() {
        super(Elevator.EXTENSION_STATE.MID, (Elevator.EXTENSION_STATE.MIN.getExtension() + Elevator.EXTENSION_STATE.MID.getExtension()) / 2d, Elevator.EXTENSION_STATE.MID.getExtension());
    }
}
