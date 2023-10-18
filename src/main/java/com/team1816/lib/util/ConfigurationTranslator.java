package com.team1816.lib.util;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.team1816.lib.hardware.components.motor.configurations.*;
import com.team1816.lib.util.logUtil.GreenLogger;

import java.util.Arrays;

import static com.team1816.lib.util.Util.closestTo;

/**
 * Utility for translating between vendordep configuration classes and 1816 proprietary classes
 *
 */
public class ConfigurationTranslator {
    //Also known as: Switch statements: the class!
    private ConfigurationTranslator() {}
    /**
     * Translates 1816 FeedbackDeviceType to TalonFXFeedbackDevice
     *
     * @see TalonFXFeedbackDevice
     * @see FeedbackDeviceType
     * @param deviceType The generalized device type
     * @return The translated device type
     */
    public static TalonFXFeedbackDevice toTalonFXFeedbackDevice(FeedbackDeviceType deviceType) {
        TalonFXFeedbackDevice talonFXFeedbackDevice;
        switch (deviceType) {
            case NO_SENSOR -> talonFXFeedbackDevice = TalonFXFeedbackDevice.None;
            case INTEGRATED_SENSOR -> talonFXFeedbackDevice = TalonFXFeedbackDevice.IntegratedSensor;
            case SENSOR_SUM -> talonFXFeedbackDevice = TalonFXFeedbackDevice.SensorSum;
            case SENSOR_DIFFERENCE -> talonFXFeedbackDevice = TalonFXFeedbackDevice.SensorDifference;
            case REMOTE_SENSOR_0 -> talonFXFeedbackDevice = TalonFXFeedbackDevice.RemoteSensor0;
            case REMOTE_SENSOR_1 -> talonFXFeedbackDevice = TalonFXFeedbackDevice.RemoteSensor1;
            case SOFTWARE_EMULATED_SENSOR -> talonFXFeedbackDevice = TalonFXFeedbackDevice.SoftwareEmulatedSensor;
            default -> {
                talonFXFeedbackDevice = TalonFXFeedbackDevice.None;
                GreenLogger.log("Attempted application of non-applicable feedback device type " + deviceType + " to TalonFX, defaulting to No sensor");
            }
        }
        return talonFXFeedbackDevice;
    }

    /**
     * Translates 1816 FeedbackDeviceType to TalonSRXFeedbackDevice
     *
     * @see TalonSRXFeedbackDevice
     * @see FeedbackDeviceType
     * @param deviceType The generalized device type
     * @return The translated device type
     */
    public static TalonSRXFeedbackDevice toTalonSRXFeedbackDevice(FeedbackDeviceType deviceType) {
        TalonSRXFeedbackDevice talonSRXFeedbackDevice;
        switch (deviceType) {
            case NO_SENSOR -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.None;
            case SENSOR_SUM -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.SensorSum;
            case SENSOR_DIFFERENCE -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.SensorDifference;
            case REMOTE_SENSOR_0 -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.RemoteSensor0;
            case REMOTE_SENSOR_1 -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.RemoteSensor1;
            case SOFTWARE_EMULATED_SENSOR -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.SoftwareEmulatedSensor;
            case ANALOG -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.Analog;
            case TACHOMETER -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.Tachometer;
            case PULSE_WIDTH -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.PulseWidthEncodedPosition;
            case ABSOLUTE_MAG_ENCODER -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute;
            case RELATIVE_MAG_ENCODER -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative;
            case QUADRATURE -> talonSRXFeedbackDevice = TalonSRXFeedbackDevice.QuadEncoder;
            default -> {
                talonSRXFeedbackDevice = TalonSRXFeedbackDevice.None;
                GreenLogger.log("Attempted application of non-applicable feedback device type " + deviceType + " to TalonSRX, defaulting to No sensor");
            }
        }
        return talonSRXFeedbackDevice;
    }

    /**
     * Translates 1816 FeedbackDeviceType to CTRE RemoteFeedbackDevice
     *
     * @see RemoteFeedbackDevice
     * @see FeedbackDeviceType
     * @param deviceType The generalized device type
     * @return The translated device type
     */
    public static RemoteFeedbackDevice toRemoteFeedbackDevice(FeedbackDeviceType deviceType) {
        RemoteFeedbackDevice remoteFeedbackDevice;
        switch (deviceType) {
            case NO_SENSOR -> remoteFeedbackDevice = RemoteFeedbackDevice.None;
            case SENSOR_SUM -> remoteFeedbackDevice = RemoteFeedbackDevice.SensorSum;
            case SENSOR_DIFFERENCE -> remoteFeedbackDevice = RemoteFeedbackDevice.SensorDifference;
            case REMOTE_SENSOR_0 -> remoteFeedbackDevice = RemoteFeedbackDevice.RemoteSensor0;
            case REMOTE_SENSOR_1 -> remoteFeedbackDevice = RemoteFeedbackDevice.RemoteSensor1;
            case SOFTWARE_EMULATED_SENSOR -> remoteFeedbackDevice = RemoteFeedbackDevice.SoftwareEmulatedSensor;
            default -> {
                remoteFeedbackDevice = RemoteFeedbackDevice.None;
                GreenLogger.log("Attempted application of non-applicable feedback device type " + deviceType + " to VictorSPX, defaulting to No sensor");
            }
        }
        return remoteFeedbackDevice;
    }

    /**
     * Translates 1816 FeedbackDeviceType to REV SparkMaxRelativeEncoder.Type
     *
     * @see SparkMaxRelativeEncoder.Type
     * @see FeedbackDeviceType
     * @param deviceType The generalized device type
     * @return The translated encoder type
     */
    public static SparkMaxRelativeEncoder.Type toSparkMaxRelativeEncoderType(FeedbackDeviceType deviceType) {
        SparkMaxRelativeEncoder.Type encoderType;
        switch (deviceType) {
            case QUADRATURE -> encoderType = SparkMaxRelativeEncoder.Type.kQuadrature;
            case HALL_SENSOR -> encoderType = SparkMaxRelativeEncoder.Type.kHallSensor;
            case NO_SENSOR -> encoderType = SparkMaxRelativeEncoder.Type.kNoSensor;
            default -> {
                GreenLogger.log("Non-SparkMax encoder type " + deviceType + " cannot be applied to SparkMaxRelativeEncoder, defaulting to No sensor.");
                encoderType = SparkMaxRelativeEncoder.Type.kNoSensor;
            }
        }
        return encoderType;
    }

    /**
     * Translates 1816 PeriodicStatusFrame to CTRE StatusFrameEnhanced
     *
     * @see PeriodicStatusFrame
     * @see StatusFrameEnhanced
     * @param frame The generalized status frame
     * @return The translated status frame
     */
    public static StatusFrameEnhanced toStatusFrameEnhanced(PeriodicStatusFrame frame) {
        StatusFrameEnhanced statusFrameEnhanced;
        switch (frame) {
            //We love super long switch statements
            case STATUS_1 -> statusFrameEnhanced = StatusFrameEnhanced.Status_1_General;
            case STATUS_2 -> statusFrameEnhanced = StatusFrameEnhanced.Status_2_Feedback0;
            case STATUS_3 -> statusFrameEnhanced = StatusFrameEnhanced.Status_3_Quadrature;
            case STATUS_4 -> statusFrameEnhanced = StatusFrameEnhanced.Status_4_AinTempVbat;
            case STATUS_6 -> statusFrameEnhanced = StatusFrameEnhanced.Status_6_Misc;
            case STATUS_7_COMMSTATUS ->statusFrameEnhanced = StatusFrameEnhanced.Status_7_CommStatus;
            case STATUS_8_PULSEWIDTH ->statusFrameEnhanced = StatusFrameEnhanced.Status_8_PulseWidth;
            case STATUS_9_MOTPROFBUFFER ->statusFrameEnhanced = StatusFrameEnhanced.Status_9_MotProfBuffer;
            case STATUS_10_TARGETS ->statusFrameEnhanced = StatusFrameEnhanced.Status_10_Targets;
            case STATUS_11_UARTGADGETEER ->statusFrameEnhanced = StatusFrameEnhanced.Status_11_UartGadgeteer;
            case STATUS_12_FEEDBACK1 ->statusFrameEnhanced = StatusFrameEnhanced.Status_12_Feedback1;
            case STATUS_13_BASE_PIDF0 ->statusFrameEnhanced = StatusFrameEnhanced.Status_13_Base_PIDF0;
            case STATUS_14_TURN_PIDF1 ->statusFrameEnhanced = StatusFrameEnhanced.Status_14_Turn_PIDF1;
            case STATUS_15_FIRMWAREAPISTATUS ->statusFrameEnhanced = StatusFrameEnhanced.Status_15_FirmwareApiStatus;
            case STATUS_21_FEEDBACKINTEGRATED->statusFrameEnhanced = StatusFrameEnhanced.Status_21_FeedbackIntegrated;
            case STATUS_BRUSHLESS_CURRENT -> statusFrameEnhanced = StatusFrameEnhanced.Status_Brushless_Current;
            default -> {
                statusFrameEnhanced = StatusFrameEnhanced.Status_1_General;
                GreenLogger.log("Attempted application of non-applicable status frame " + frame + " to a Talon motor, defaulting to Status 1 (General)");
            }
        }
        return statusFrameEnhanced;
    }

    //I LOVE VICTORSPX NOT USING NEW CONFIGURATION ENUMS!!!!!!
    /**
     * Translates 1816 PeriodicStatusFrame to CTRE StatusFrame
     *
     * @see PeriodicStatusFrame
     * @see StatusFrame
     * @param frame The generalized status frame
     * @return The translated status frame
     */
    public static StatusFrame toStatusFrame(PeriodicStatusFrame frame) {
        StatusFrame statusFrame;
        switch (frame) {
            case STATUS_1 -> statusFrame = StatusFrame.Status_1_General;
            case STATUS_2 -> statusFrame = StatusFrame.Status_2_Feedback0;
            case STATUS_4 -> statusFrame = StatusFrame.Status_4_AinTempVbat;
            case STATUS_6 -> statusFrame = StatusFrame.Status_6_Misc;
            case STATUS_7_COMMSTATUS -> statusFrame = StatusFrame.Status_7_CommStatus;
            case STATUS_9_MOTPROFBUFFER -> statusFrame = StatusFrame.Status_9_MotProfBuffer;
            case STATUS_10_TARGETS -> statusFrame = StatusFrame.Status_10_Targets;
            case STATUS_12_FEEDBACK1 -> statusFrame = StatusFrame.Status_12_Feedback1;
            case STATUS_13_BASE_PIDF0 -> statusFrame = StatusFrame.Status_13_Base_PIDF0;
            case STATUS_14_TURN_PIDF1 -> statusFrame = StatusFrame.Status_14_Turn_PIDF1;
            case STATUS_15_FIRMWAREAPISTATUS -> statusFrame = StatusFrame.Status_15_FirmwareApiStatus;
            case STATUS_17_TARGETS1 -> statusFrame = StatusFrame.Status_17_Targets1;
            default -> {
                statusFrame = StatusFrame.Status_1_General;
                GreenLogger.log("Attempted application of non-applicable status frame " + frame + " to a VictorSPX, defaulting to Status 1 (General)");
            }
        }
        return statusFrame;
    }

    public static CANSparkMaxLowLevel.PeriodicFrame toPeriodicFrame(PeriodicStatusFrame frame) {
        CANSparkMaxLowLevel.PeriodicFrame periodicFrame;
        switch (frame) {
            case STATUS_0 -> periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus0;
            case STATUS_1 -> periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus1;
            case STATUS_2 -> periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus2;
            case STATUS_3 -> periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus3;
            case STATUS_4 -> periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus4;
            case STATUS_5 -> periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus5;
            case STATUS_6 -> periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus6;
            default -> {
                GreenLogger.log("Cannot apply periodic frame status " + frame + " to SparkMax PeriodicFrame, defaulting to status 0");
                periodicFrame = CANSparkMaxLowLevel.PeriodicFrame.kStatus0;
            }
        }
        return periodicFrame;
    }

    /**
     * Translates period milliseconds into a CTRE SensorVelocityMeasPeriod object
     *
     * @see SensorVelocityMeasPeriod
     * @param periodms
     * @return The translated object
     */
    public static SensorVelocityMeasPeriod toSensorVelocityMeasPeriod(int periodms) {
        Integer[] possibilities = {1, 2, 5, 10, 20, 50, 100};
        int closestPossible = closestTo(possibilities, periodms);

        if (!Arrays.asList(possibilities).contains(periodms)) {
            GreenLogger.log("Velocity measurement period converted from " + periodms + " ms to closest possible value, " + closestPossible + " ms");
        }

        SensorVelocityMeasPeriod sensorVelocityMeasPeriod;
        switch (closestPossible){
            case 1 -> sensorVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_1Ms;
            case 2 -> sensorVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_2Ms;
            case 5 -> sensorVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_5Ms;
            case 10 -> sensorVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_10Ms;
            case 50 -> sensorVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_50Ms;
            case 100 -> sensorVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_100Ms;
            default -> sensorVelocityMeasPeriod = SensorVelocityMeasPeriod.Period_20Ms; // 20 is our default
        }
        return sensorVelocityMeasPeriod;
    }

    /**
     * Translates 1816 GreenControlMode into CTRE ControlMode
     *
     * @see ControlMode
     * @see GreenControlMode
     * @param controlMode The generalized control mode
     * @return The translated control mode
     */
    public static ControlMode toCTREControlMode(GreenControlMode controlMode) {
        ControlMode CTREControlMode;
        switch (controlMode) {
            case PERCENT_OUTPUT -> CTREControlMode = ControlMode.PercentOutput;
            case VELOCITY_CONTROL -> CTREControlMode = ControlMode.Velocity;
            case POSITION_CONTROL -> CTREControlMode = ControlMode.Position;
            case MOTION_PROFILE -> CTREControlMode = ControlMode.MotionProfile;
            case CURRENT -> CTREControlMode = ControlMode.Current;
            case FOLLOWER -> CTREControlMode = ControlMode.Follower;
            case MOTION_MAGIC -> CTREControlMode = ControlMode.MotionMagic;
            case MOTION_PROFILE_ARC -> CTREControlMode = ControlMode.MotionProfileArc;
            case MUSIC_TONE -> CTREControlMode = ControlMode.MusicTone;
            case DISABLED -> CTREControlMode = ControlMode.Disabled;
            default -> {
                GreenLogger.log("REV-Exclusive Control mode " + controlMode + " cannot be set to TalonFX, defaulting to Percent-Output");
                CTREControlMode = ControlMode.PercentOutput;
            }
        }
        return CTREControlMode;
    }

    public static CANSparkMax.ControlType toSparkMaxControlType(GreenControlMode controlMode) {
        CANSparkMax.ControlType controlType;
        switch (controlMode) {
            case PERCENT_OUTPUT -> controlType = CANSparkMax.ControlType.kDutyCycle;
            case VELOCITY_CONTROL -> controlType = CANSparkMax.ControlType.kVelocity;
            case POSITION_CONTROL -> controlType = CANSparkMax.ControlType.kPosition;
            case MOTION_PROFILE -> controlType = CANSparkMax.ControlType.kSmartMotion;
            case CURRENT -> controlType = CANSparkMax.ControlType.kCurrent;
            case VOLTAGE_CONTROL -> controlType = CANSparkMax.ControlType.kVoltage;
            case SMART_VELOCITY -> controlType = CANSparkMax.ControlType.kSmartVelocity;
            default -> {
                GreenLogger.log("Motor Control Mode " + controlMode + " not applicable to SparkMax ControlType, defaulting to Percent-Output");
                controlType = CANSparkMax.ControlType.kDutyCycle;
            }
        }
        return controlType;
    }

    /**
     * Translates CTRE ControlMode into 1816 GreenControlMode
     * @see ControlMode
     * @see GreenControlMode
     *
     * @param controlMode The CTRE Control Mode
     * @return The generalized translation
     */
    public static GreenControlMode toGreenControlMode(ControlMode controlMode) {
        GreenControlMode greenControlMode;
        switch (controlMode) {
            case PercentOutput -> greenControlMode = GreenControlMode.PERCENT_OUTPUT;
            case Velocity -> greenControlMode = GreenControlMode.VELOCITY_CONTROL;
            case Position -> greenControlMode = GreenControlMode.POSITION_CONTROL;
            case MotionProfile -> greenControlMode = GreenControlMode.MOTION_PROFILE;
            case Current -> greenControlMode = GreenControlMode.CURRENT;
            case Follower -> greenControlMode = GreenControlMode.FOLLOWER;
            case MotionMagic -> greenControlMode = GreenControlMode.MOTION_MAGIC;
            case MotionProfileArc -> greenControlMode = GreenControlMode.MOTION_PROFILE_ARC;
            case MusicTone -> greenControlMode = GreenControlMode.MUSIC_TONE;
            default -> greenControlMode = GreenControlMode.DISABLED;
        }
        return greenControlMode;
    }

    /**
     * Translates 1816 MotionCurveType into an int
     *
     * @param motionCurveType The Generalized motion curve type
     * @param motionCurveStrength The strength of the motion curve; 0 for trapezoidal, [1,8] for S-curve
     * @see MotionCurveType
     * @return The motion curve type as an int
     */
    public static int toMotionCurveInt(MotionCurveType motionCurveType, int motionCurveStrength) {
        if (motionCurveType == MotionCurveType.S_CURVE && motionCurveStrength > 0) {
            return motionCurveStrength;
        }
        return  0;
    }

}
