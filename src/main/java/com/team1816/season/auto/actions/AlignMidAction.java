package com.team1816.season.auto.actions;

import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.season.subsystems.Elevator;

public class AlignMidAction extends SeriesAction {
    public AlignMidAction() {
        super(new AlignAction(Elevator.EXTENSION_STATE.MID, (Elevator.EXTENSION_STATE.MIN.getExtension() + Elevator.EXTENSION_STATE.MID.getExtension()) / 2d, Elevator.EXTENSION_STATE.MID.getExtension()));
    }
}
