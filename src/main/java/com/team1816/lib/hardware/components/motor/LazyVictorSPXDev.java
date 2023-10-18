package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team1816.lib.hardware.components.motor.configurations.*;
import com.team1816.lib.util.ConfigurationTranslator;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyVictorSPXDev extends VictorSPX implements IGreenMotorDev {
    protected double lastSet = Double.NaN;
    protected String name = "";
    protected ControlMode lastControlMode = null;

    protected Faults faults;
    protected StickyFaults stickyFaults;

    protected boolean isFollower;

    protected SoftLimitStatus softLimitStatus;

    protected double arbitraryFeedForward = 0;

    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public LazyVictorSPXDev(int deviceNumber, String motorName) {
        super(deviceNumber);
        name = motorName;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public MotorType get_MotorType() {
        return MotorType.VICTORSPX;
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType) {
        selectFeedbackSensor(deviceType, 0);
    }

    public void selectFeedbackSensor(FeedbackDeviceType deviceType, int closedLoopSlotID) {
        super.configSelectedFeedbackSensor(
            ConfigurationTranslator.toRemoteFeedbackDevice(deviceType),
            closedLoopSlotID,
            Constants.kCANTimeoutMs
        );
    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration) {
        GreenLogger.log("Current Limits nonexistent for VictorSPX!!!");
    }

    @Override
    public void configCurrentLimit(int current) {
        GreenLogger.log("Current Limits nonexistent for VictorSPX!!!");
    }

    @Override
    public void setPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame, int periodms) {
        super.setStatusFramePeriod(
            ConfigurationTranslator.toStatusFrame(statusFrame),
            periodms
        );
    }

    @Override
    public int getPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame) {
        return super.getStatusFramePeriod(
            ConfigurationTranslator.toStatusFrame(statusFrame)
        );
    }

    @Override
    public double getOutputCurrent() {
        return super.getOutputCurrent(); // Deprecated & victors are so old that they don't have getSupplyCurrent() or getStatorCurrent() smile
    }

    @Override
    public void setVelocityMeasurementPeriod(int periodms) {
        super.configVelocityMeasurementPeriod(
            ConfigurationTranslator.toSensorVelocityMeasPeriod(periodms)
        );
    }

    @Override
    public void set(GreenControlMode controlMode, double demand) {
        ControlMode mode = ConfigurationTranslator.toCTREControlMode(controlMode);
        if (demand != lastSet || mode != lastControlMode) {
            if (!super.hasResetOccurred()) {
                lastSet = demand;
                lastControlMode = mode;
                super.set(mode, demand, DemandType.ArbitraryFeedForward, arbitraryFeedForward); //Note that arbitraryFF is initialized at 0
            } else {
                DriverStation.reportError("MOTOR " + getDeviceID() + " HAS RESET", false);
            }
        }
    }

    @Override
    public void configForwardLimitSwitch(boolean normallyOpen) {
        LimitSwitchNormal openOrClosed = normallyOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed;
        super.configForwardLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteCANifier,
            openOrClosed,
            Constants.kCANTimeoutMs
        );
    }

    @Override
    public void configReverseLimitSwitch(boolean normallyOpen) {
        LimitSwitchNormal openOrClosed = normallyOpen ? LimitSwitchNormal.NormallyOpen : LimitSwitchNormal.NormallyClosed;
        super.configReverseLimitSwitchSource(
            RemoteLimitSwitchSource.RemoteCANifier,
            openOrClosed,
            Constants.kCANTimeoutMs
        );
    }

    @Override
    public void configOpenLoopRampRate(int secondsNeutralToFull) {
        super.configOpenloopRamp(secondsNeutralToFull);
    }

    @Override
    public void configClosedLoopRampRate(int secondsNeutralToFull) {
        super.configClosedloopRamp(secondsNeutralToFull);
    }

    @Override
    public void config_PeakOutputForward(double percentOut) {
        super.configPeakOutputForward(percentOut);
    }

    @Override
    public void config_PeakOutputReverse(double percentOut) {
        super.configPeakOutputReverse(percentOut);
    }

    @Override
    public void config_NominalOutputForward(double percentOut) {
        super.configNominalOutputForward(percentOut);
    }

    @Override
    public void config_NominalOutputReverse(double percentOut) {
        super.configNominalOutputReverse(percentOut);
    }

    @Override
    public void config_NeutralDeadband(double deadbandPercent) {
        super.configNeutralDeadband(deadbandPercent);
    }

    @Override
    public void configVoltageCompensation(double voltage) {
        super.configVoltageCompSaturation(voltage);
    }

    @Override
    public double getMotorTemperature() {
        return super.getTemperature();
    }

    @Override
    public double getSensorPosition(int closedLoopSlotID) {
        return super.getSelectedSensorPosition(closedLoopSlotID);
    }

    @Override
    public double getSensorVelocity(int closedLoopSlotID) {
        return super.getSelectedSensorVelocity(closedLoopSlotID);
    }

    @Override
    public void setSensorPosition(double sensorPosition, int closedLoopSlotID) {
        super.setSelectedSensorPosition(sensorPosition, closedLoopSlotID, Constants.kCANTimeoutMs);
    }

    @Override
    public void enableLimitSwitches(boolean isEnabled) {
        super.overrideLimitSwitchesEnable(isEnabled);
    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit) {
        super.configForwardSoftLimitThreshold(forwardSoftLimit);
    }

    @Override
    public void configReverseSlotLimit(double reverseSoftLimit) {
        super.configReverseSoftLimitThreshold(reverseSoftLimit);
    }

    @Override
    public void enableForwardSoftLimit(boolean isEnabled) {
        super.configForwardSoftLimitEnable(isEnabled);
        softLimitStatus = updateSoftLimitStatus(
            softLimitStatus,
            isEnabled ? SoftLimitStatus.FORWARD : SoftLimitStatus.FORWARD_DISABLE
        );
    }

    @Override
    public void enableReverseSoftLimit(boolean isEnabled) {
        super.configReverseSoftLimitEnable(isEnabled);
        softLimitStatus = updateSoftLimitStatus(
            softLimitStatus,
            isEnabled ? SoftLimitStatus.REVERSE : SoftLimitStatus.REVERSE_DISABLE
        );
    }

    @Override
    public void set_kP(int pidSlotID, double kP) {
        super.config_kP(pidSlotID, kP);
    }

    @Override
    public void set_kI(int pidSlotID, double kI) {
        super.config_kI(pidSlotID, kI);
    }

    @Override
    public void set_kD(int pidSlotID, double kD) {
        super.config_kD(pidSlotID, kD);
    }

    @Override
    public void set_kF(int pidSlotID, double kF) {
        super.config_kF(pidSlotID, kF);
    }

    @Override
    public void setArbitraryFeedForward(double feedForward) {
        arbitraryFeedForward = feedForward;
    }

    @Override
    public double getFeedForward(int closedLoopSlotID, int PIDSlotID) {
        return super.getActiveTrajectoryArbFeedFwd(closedLoopSlotID);
    }

    @Override
    public void selectPIDSlot(int pidSlotID, int closedLoopSlotID) {
        super.selectProfileSlot(pidSlotID, closedLoopSlotID);
    }

    @Override
    public void set_iZone(int pidSlotID, double iZone) {
        super.config_IntegralZone(pidSlotID, iZone);
    }

    @Override
    public void configAllowableErrorClosedLoop(int pidSlotID, double allowableError) {
        super.configAllowableClosedloopError(pidSlotID, allowableError);
    }

    @Override
    public void setMaxIAccumulation(int pidSlotID, double maxIAccum) {
        super.configMaxIntegralAccumulator(pidSlotID, maxIAccum);
    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput) {
        super.configClosedLoopPeakOutput(pidSlotID, peakOutput);
    }

    @Override
    public void setIAccumulation(int closedLoopSlotID, double IAccum) {
        super.setIntegralAccumulator(IAccum, closedLoopSlotID, Constants.kCANTimeoutMs);
    }

    @Override
    public double getIAccum(int closedLoopSlotID) {
        return super.getIntegralAccumulator(closedLoopSlotID);
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity) {
        super.configMotionCruiseVelocity(maxVelocity);
    }

    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration) {
        super.configMotionAcceleration(maxAcceleration);
    }

    @Override
    public void configMotionCurve(MotionCurveType motionCurveType, int curveStrength) {
        if (curveStrength > 8) {
            GreenLogger.log("Motion Curve Strength cannot exceed 8, adjusting down.");
            curveStrength = 8;
        } else if (curveStrength < 0) {
            GreenLogger.log("Motion Curve Strength cannot be negative, adjusting to 0.");
            curveStrength = 0;
        }
        super.configMotionSCurveStrength(
            ConfigurationTranslator.toMotionCurveInt(motionCurveType, curveStrength)
        );
    }

    @Override
    public void clearMotionProfileTrajectoryBuffer() {
        super.clearMotionProfileTrajectories();
    }

    @Override
    public String get_LastError() {
        return "CTRE ErrorCode " + super.getLastError().toString();
    }

    @Override
    public String get_Faults() {
        return "CTRE ErrorCode " + super.getFaults(faults).name();
    }

    @Override
    public String get_StickyFaults() {
        return "CTRE ErrorCode " + super.getStickyFaults(stickyFaults).toString();
    }

    @Override
    public GreenControlMode get_ControlMode() {
        return ConfigurationTranslator.toGreenControlMode(super.getControlMode());
    }

    @Override
    public void follow(IGreenMotorDev leader) {
        isFollower = true;
        // ONLY works to follow CTRE Motor Controllers.
        if (leader.get_MotorType() == MotorType.SPARKMAX || leader.get_MotorType() == MotorType.GHOST) {
            GreenLogger.log("TalonFX cannot follow non-CTRE motor " + leader.getName() + " of type " + leader.get_MotorType());
        } else {
            super.follow((IMotorController) leader); //I Really hope this works as intended, it SHOULD only do this when it can be cast but I'm not sure- have DDay or Mika check
        }
    }

    @Override
    public double getSupplyCurrent() {
        GreenLogger.log("getSupplyCurrent() not present for VictorSPX, using getOutputCurrent() instead.");
        return super.getOutputCurrent();
    }

    @Override
    public void restore_FactoryDefaults() {
        super.configFactoryDefault();
    }

    @Override
    public int getQuadraturePosition() {
        GreenLogger.log("Quadrature Position is nonexistent for VictorSPXs");
        return -1;

    }

    @Override
    public void setQuadraturePosition(int quadraturePosition) {
        GreenLogger.log("Quadrature Position is nonexistent for VictorSPXs");
    }

    @Override
    public int getPulseWidthPosition() {
        GreenLogger.log("Pulse Width Position is nonexistent for VictorSPXs");
        return -1;
    }

    @Override
    public boolean isFollower() {
        return isFollower;
    }

    @Override
    public SoftLimitStatus getSoftLimitStatus() {
        return softLimitStatus;
    }

    @Override
    public void configControlFramePeriod(ControlFrame controlFrame, int periodms) {
        super.setControlFramePeriod(controlFrame, periodms);
    }
}
