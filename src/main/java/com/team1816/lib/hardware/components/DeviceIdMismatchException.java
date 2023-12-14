package com.team1816.lib.hardware.components;

/**
 * Exception thrown when a device's id does not match to its given ID in yaml
 *
 */
public class DeviceIdMismatchException extends Exception {

    public DeviceIdMismatchException() {
        super();
    }
    public DeviceIdMismatchException(String deviceName) {
        super("device id of " + deviceName + " does not match yml configuration");
    }

    private static final long serialVersionUID = 6164557824453626058L;

}
