package com.team1816.lib.hardware.components.motor.configurations;

/**
 * Enum containing all motor feedback device types
 *
 * @see com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice
 * @see com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice
 * @see com.revrobotics.SparkMaxRelativeEncoder.Type
 */
public enum FeedbackDeviceType { // No enum inheritance :(
    // Universal
    NO_SENSOR,

    // CTRE Universal
    SENSOR_SUM,
    SENSOR_DIFFERENCE,
    REMOTE_SENSOR_0,
    REMOTE_SENSOR_1,
    SOFTWARE_EMULATED_SENSOR,

    // TalonFX
    INTEGRATED_SENSOR,

    // TalonSRX
    ANALOG,
    TACHOMETER,
    PULSE_WIDTH,
    ABSOLUTE_MAG_ENCODER,
    RELATIVE_MAG_ENCODER,

    // TalonSRX & SparkMax
    QUADRATURE,

    // SparkMax
    HALL_SENSOR
}
