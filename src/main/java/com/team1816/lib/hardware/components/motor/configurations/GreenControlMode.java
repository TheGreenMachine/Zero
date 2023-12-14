package com.team1816.lib.hardware.components.motor.configurations;

/**
 * A general enum containing motor control modes
 *
 * @see com.ctre.phoenix.motorcontrol.ControlMode
 * @see com.revrobotics.CANSparkMax.ControlType
 */
public enum GreenControlMode {
    // Universal
    PERCENT_OUTPUT, //TODO when phoenix 6 is brought in, change to DUTY_CYCLE
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
