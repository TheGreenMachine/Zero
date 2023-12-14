package com.team1816.lib.legacy;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.util.logUtil.GreenLogger;

/**
 * This class is a thin wrapper around the CANSparkMax that reduces CAN bus / CPU overhead.
 * Connects with REV SparkMax motor controllers and adapts it for the universal IGreenMotor.
 *
 * @see IGreenMotor
 * @see CANSparkMax
 */
@Deprecated
public class LegacyLazySparkMax implements LegacyIGreenMotor {

    private CANSparkMax motor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;

    protected double lastSet = Double.NaN;
    protected String name = "";
    protected CANSparkMax.ControlType lastControlMode = null;

    public LegacyLazySparkMax(int deviceNumber, String motorName) {
        motor = new CANSparkMax(deviceNumber, CANSparkMaxLowLevel.MotorType.kBrushless);
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        name = motorName;
    }

    @Override
    public void set(ControlMode mode, double demand) {
        canMaxSet(mode, demand);
    }

    @Override
    public void set(
        ControlMode mode,
        double demand0,
        DemandType demand1Type,
        double demand1
    ) {
        canMaxSet(mode, demand0);
    }

    private void canMaxSet(ControlMode mode, double demand) {
        CANSparkMax.ControlType controlType = convertControlMode(mode);
        if (demand != lastSet || controlType != lastControlMode) {
            lastSet = demand;
            lastControlMode = controlType;
            pidController.setReference(demand, controlType); // note that this uses rpm for velocity!
        }
    }

    public CANSparkMax getSpark() {
        return motor;
    }

    @Override
    public void neutralOutput() {
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        motor.setIdleMode(neutralMode == NeutralMode.Brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    @Override
    public void setInverted(InvertType invertType) {
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        motor.setOpenLoopRampRate(secondsFromNeutralToFull);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedloopRamp(
        double secondsFromNeutralToFull,
        int timeoutMs
    ) {
        motor.setClosedLoopRampRate(secondsFromNeutralToFull);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configVoltageMeasurementFilter(
        int filterWindowSamples,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public void enableVoltageCompensation(boolean enable) {
    }

    @Override
    public double getBusVoltage() {
        return 0;
    }

    @Override
    public double getMotorOutputPercent() {
        return 0;
    }

    @Override
    public double getMotorOutputVoltage() {
        return 0;
    }

    @Override
    public double getTemperature() {
        return motor.getMotorTemperature();
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(
        RemoteFeedbackDevice feedbackDevice,
        int pidIdx,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configSelectedFeedbackCoefficient(
        double coefficient,
        int pidIdx,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(
        int deviceID,
        RemoteSensorSource remoteSensorSource,
        int remoteOrdinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(
        CANCoder canCoderRef,
        int remoteOrdinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configRemoteFeedbackFilter(
        BaseTalon talonRef,
        int remoteOrdinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configSensorTerm(
        SensorTerm sensorTerm,
        FeedbackDevice feedbackDevice,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public double getSelectedSensorPosition(int pidIdx) {
        return encoder.getPosition();
    } // native position value

    @Override
    public double getSelectedSensorVelocity(int pidIdx) {
        return encoder.getVelocity();
    } // RPM

    @Override
    public ErrorCode setSelectedSensorPosition(
        double sensorPos,
        int pidIdx,
        int timeoutMs
    ) {
        encoder.setPosition(sensorPos);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) {
        return null;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrame frame,
        int periodMs,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(
        RemoteLimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int deviceID,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        RemoteLimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int deviceID,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public void overrideLimitSwitchesEnable(boolean enable) {
    }

    @Override
    public ErrorCode configForwardSoftLimitThreshold(
        double forwardSensorLimit,
        int timeoutMs
    ) {
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)forwardSensorLimit);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(
        double reverseSensorLimit,
        int timeoutMs
    ) {
        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)reverseSensorLimit); //TODO make sure that reverseSensorLimit doesn't need to be negative or something
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) {
        return null;
    }

    @Override
    public void overrideSoftLimitsEnable(boolean enable) {
    }

    @Override
    public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) {
        pidController.setP(value, slotIdx);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) {
        pidController.setI(value, slotIdx);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) {
        pidController.setD(value, slotIdx);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) {
        pidController.setFF(value, slotIdx); // Feedforward gain
        return ErrorCode.OK;
    }

    public ErrorCode config_Pid_Manual(int slotIdx, SlotConfiguration slotConfiguration) {
        pidController.setP(slotConfiguration.kP, slotIdx);
        pidController.setI(slotConfiguration.kI, slotIdx);
        pidController.setD(slotConfiguration.kD, slotIdx);
        pidController.setFF(slotConfiguration.kF, slotIdx);
        pidController.setIZone(slotConfiguration.integralZone);
        pidController.setSmartMotionAllowedClosedLoopError(slotConfiguration.allowableClosedloopError, slotIdx);
        return ErrorCode.OK;
    }
    @Override
    public ErrorCode config_IntegralZone(int slotIdx, double izone, int timeoutMs) {
        pidController.setIZone(izone, slotIdx);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configAllowableClosedloopError(
        int slotIdx,
        double allowableCloseLoopError,
        int timeoutMs
    ) {
        pidController.setSmartMotionAllowedClosedLoopError(allowableCloseLoopError, slotIdx); //TODO make sure this is actually allowable error
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configMaxIntegralAccumulator(
        int slotIdx,
        double iaccum,
        int timeoutMs
    ) {
        pidController.setIMaxAccum(iaccum, slotIdx);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configClosedLoopPeakOutput(
        int slotIdx,
        double percentOut,
        int timeoutMs
    ) {
        pidController.setOutputRange(0, percentOut, slotIdx); //TODO MAKE SURE THIS WORKS
        return null;
    }

    @Override
    public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) {
        return ErrorCode.OK;
    }

    @Override
    public double getClosedLoopError(int pidIdx) {
        return 0;
    }

    @Override
    public double getIntegralAccumulator(int pidIdx) {
        return 0;
    }

    @Override
    public double getErrorDerivative(int pidIdx) {
        return 0;
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
    }

    @Override
    public double getClosedLoopTarget(int pidIdx) {
        return 0;
    }

    @Override
    public double getActiveTrajectoryPosition() {
        return 0;
    }

    @Override
    public double getActiveTrajectoryVelocity() {
        return 0;
    }

    @Override
    public ErrorCode configMotionCruiseVelocity(
        double sensorUnitsPer100ms,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configMotionAcceleration(
        double sensorUnitsPer100msPerSec,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configMotionProfileTrajectoryPeriod(
        int baseTrajDurationMs,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode clearMotionProfileTrajectories() {
        return null;
    }

    @Override
    public int getMotionProfileTopLevelBufferCount() {
        return 0;
    }

    @Override
    public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) {
        return null;
    }

    @Override
    public boolean isMotionProfileTopLevelBufferFull() {
        return false;
    }

    @Override
    public void processMotionProfileBuffer() {
    }

    @Override
    public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) {
        return null;
    }

    @Override
    public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode changeMotionControlFramePeriod(int periodMs) {
        return null;
    }

    @Override
    public ErrorCode getLastError() {
        return null;
    }

    @Override
    public ErrorCode getFaults(Faults toFill) {
        return null;
    }

    @Override
    public ErrorCode getStickyFaults(StickyFaults toFill) {
        return null;
    }

    @Override
    public ErrorCode clearStickyFaults(int timeoutMs) {
        return null;
    }

    @Override
    public int getFirmwareVersion() {
        return 0;
    }

    @Override
    public boolean hasResetOccurred() {
        return false;
    }

    @Override
    public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) {
        return null;
    }

    @Override
    public int configGetCustomParam(int paramIndex, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configSetParameter(
        ParamEnum param,
        double value,
        int subValue,
        int ordinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configSetParameter(
        int param,
        double value,
        int subValue,
        int ordinal,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) {
        return 0;
    }

    @Override
    public int getBaseID() {
        return 0;
    }

    public void setSensorPhase(boolean isInverted) {
        GreenLogger.log("missing op for inverting a Rev motor!");
    }

    public int getDeviceID() {
        return motor.getDeviceId();
    }

    @Override
    public ControlMode getControlMode() {
        return null;
    }

    public static CANSparkMax.ControlType convertControlMode(ControlMode controlMode) {
        if (controlMode == ControlMode.PercentOutput) {
            return CANSparkMax.ControlType.kDutyCycle;
        } else if (controlMode == ControlMode.Velocity) {
            return CANSparkMax.ControlType.kVelocity;
        } else if (controlMode == ControlMode.Position) {
            return CANSparkMax.ControlType.kPosition;
        } else {
            GreenLogger.log("spark motor not set to a control mode!");
            return CANSparkMax.ControlType.kDutyCycle; // what should be default?
        }
    }

    @Override
    public ErrorCode configSelectedFeedbackSensor(
        FeedbackDevice feedbackDevice,
        int pidIdx,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configSupplyCurrentLimit(
        SupplyCurrentLimitConfiguration currLimitCfg,
        int timeoutMs
    ) {
        motor.setSmartCurrentLimit((int) currLimitCfg.currentLimit);
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode setStatusFramePeriod(
        StatusFrameEnhanced frame,
        int periodMs,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public int getStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) {
        return 0;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(
        SensorVelocityMeasPeriod period,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configVelocityMeasurementPeriod(
        VelocityMeasPeriod period,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configVelocityMeasurementWindow(int windowSize, int timeoutMs) {
        return null;
    }

    @Override
    public ErrorCode configForwardLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        return null;
    }

    @Override
    public ErrorCode configReverseLimitSwitchSource(
        LimitSwitchSource type,
        LimitSwitchNormal normalOpenOrClose,
        int timeoutMs
    ) {
        return null;
    }

    public CANSparkMax getMotor() {
        return motor;
    }

    @Override
    public void follow(IMotorController masterToFollow) { // only use if not inverted
        motor.follow(((LegacyLazySparkMax) masterToFollow).getMotor());
    }

    public void follow(IGreenMotor masterToFollow, boolean inverted) {
        motor.follow(((LegacyLazySparkMax) masterToFollow).getMotor(), inverted);
    }

    @Override
    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public void valueUpdated() {
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public ErrorCode configAllSettings(BaseTalonConfiguration allConfigs, int timeoutMs) {
        GreenLogger.log("WARNING: configAllSettings not working for sparkMax motors!");
        return ErrorCode.OK;
    }

    @Override
    public ErrorCode configFactoryDefault(int timeoutMs) {
        GreenLogger.log(
            "WARNING: configFactoryDefault not working for sparkMax motors!"
        );
        return ErrorCode.OK;
    }
}
