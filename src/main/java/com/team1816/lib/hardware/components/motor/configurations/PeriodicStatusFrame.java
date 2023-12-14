package com.team1816.lib.hardware.components.motor.configurations;

/**
 * Enum containing all status frame types
 *
 * @see com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
 * @see com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame
 */
public enum PeriodicStatusFrame {
    // SparkMax Exclusive
    STATUS_0,
    STATUS_5, // No, we do not know why CTRE doesn't have a status 5.

    // Talon & SparkMax
    STATUS_3,    // Talon: Quadrature sensor

    // Universal
    STATUS_1,    // CTRE: General Status
    STATUS_2,    // CTRE: Feedback for selected sensor on primary PID[0].
    STATUS_4,    // CTRE: Analog sensor, motor controller temperature, and voltage at input leads
    STATUS_6,    // CTRE: Miscellaneous signals

    // Talon Exclusive
    STATUS_8_PULSEWIDTH,
    STATUS_11_UARTGADGETEER,
    STATUS_21_FEEDBACKINTEGRATED,
    STATUS_BRUSHLESS_CURRENT,

    // Victor Exclusive
    STATUS_17_TARGETS1,

    // CTRE universal
    STATUS_7_COMMSTATUS,
    STATUS_9_MOTPROFBUFFER,
    STATUS_10_TARGETS,
    STATUS_12_FEEDBACK1,
    STATUS_13_BASE_PIDF0,
    STATUS_14_TURN_PIDF1,
    STATUS_15_FIRMWAREAPISTATUS,

}
