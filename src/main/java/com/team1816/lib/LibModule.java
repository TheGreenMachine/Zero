package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.team1816.lib.controlboard.ControlBoard;
import com.team1816.lib.controlboard.ControlUtils;
import com.team1816.lib.controlboard.Controller;
import com.team1816.lib.controlboard.IControlBoard;
import com.team1816.lib.subsystems.drive.Drive;

/**
 * Configures the lib bindings for the injector
 *
 * @see Injector
 */
public class LibModule extends AbstractModule {

    @Override
    protected void configure() {
        bind(Drive.Factory.class).to(DriveFactory.class);
        bind(Controller.Factory.class).to(ControlUtils.class);
        bind(IControlBoard.class).to(ControlBoard.class);
    }
}
