package com.team1816.lib.util.logUtil;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The universal project-wide message logging wrapper.
 * This class utilizes WPI's DataLogManager and is a wrapper for the standard System.out.println()
 *
 * @see DataLogManager
 */
public class GreenLogger {
    /**
     * Logs a string message
     *
     * @param s message
     */
    public static void log(String s) {
        if (Constants.kLoggingRobot) {
            DataLogManager.log(s);
        }
    }

    /**
     * Logs a boolean message
     *
     * @param b message
     */
    public static void log(boolean b) {
        if (Constants.kLoggingRobot) {
            DataLogManager.log(String.valueOf(b));
        }
    }

    /**
     * Logs an exception message
     *
     * @param e exception
     */
    public static void log(Exception e) {
        System.out.println(e.getMessage());
    }
}
