package com.team1816.example;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.commands.AutoCommand;

/**
 * An auto command works almost like an auto mode.
 */
public class ExampleCommand extends AutoCommand {
    public ExampleCommand() {}

    public void routine() throws AutoModeEndedException {}
}
