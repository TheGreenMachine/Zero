package com.team1816.lib.hardware.components.motor.configurations;

/**
 * A general enum containing motor control modes
 *
 * @see com.ctre.phoenix.motorcontrol.ControlMode
 * @see com.revrobotics.CANSparkMax.ControlType
 */
public enum GreenControlMode {
    // Universal
    PERCENT_OUTPUT,
    VELOCITY_CONTROL,
    POSITION_CONTROL,
    MOTION_PROFILE,
    CURRENT,

    // CTRE Exclusive
    FOLLOWER,
    MOTION_MAGIC,
    MOTION_PROFILE_ARC,
    MUSIC_TONE,
    DISABLED,

    // REV Exclusive
    VOLTAGE_CONTROL,
    SMART_VELOCITY,

}
