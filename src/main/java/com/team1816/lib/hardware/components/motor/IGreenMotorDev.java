package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1816.lib.hardware.components.motor.configurations.*;

public interface IGreenMotorDev {
    //TODO sort & annotate

    //Some getter methods add an underscore after get to avoid conflicts in LazyMotor classes

    String getName();

    MotorType get_MotorType();

    void selectFeedbackSensor(FeedbackDeviceType deviceType);

    void configCurrentLimit(SupplyCurrentLimitConfiguration configuration);

    void configCurrentLimit(int current);

    void setPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame, int periodms);

    int getPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame);

    double getOutputCurrent();

    void setVelocityMeasurementPeriod(int periodms);

    void set(GreenControlMode controlMode, double demand);

    void configForwardLimitSwitch(boolean normallyOpen);

    void configReverseLimitSwitch(boolean normallyOpen);

    void neutralOutput();

    void setNeutralMode(NeutralMode neutralMode);

    void setSensorPhase(boolean isInverted);

    void setInverted(boolean isInverted);

    boolean getInverted();

    void configOpenLoopRampRate(int secondsNeutralToFull);

    void configClosedLoopRampRate(int secondsNeutralToFull);

    void config_PeakOutputForward(double percentOut);

    void config_PeakOutputReverse(double percentOut);

    void config_NominalOutputForward(double percentOut);

    void config_NominalOutputReverse(double percentOut);

    void config_NeutralDeadband(double deadbandPercent);

    void configVoltageCompensation(double voltage);

    void enableVoltageCompensation(boolean isEnabled);

    double getBusVoltage();

    double getMotorOutputPercent();

    double getMotorOutputVoltage();

    double getMotorTemperature();

    double getSensorPosition(int closedLoopSlotID);

    double getSensorVelocity(int closedLoopSlotID);

    void setSensorPosition(double sensorPosition, int closedLoopSlotID);

    void enableLimitSwitches(boolean isEnabled);

    void configForwardSoftLimit(double forwardSoftLimit);

    void configReverseSlotLimit(double reverseSoftLimit);

    void enableForwardSoftLimit(boolean isEnabled);

    void enableReverseSoftLimit(boolean isEnabled);

    void set_kP(int pidSlotID, double kP);

    void set_kI(int pidSlotID, double kI);

    void set_kD(int pidSlotID, double kD);

    void set_kF(int pidSlotID, double kF);

    void setArbitraryFeedForward(double feedForward);

    double getFeedForward(int closedLoopSlotID, int pidSlotID);

    void selectPIDSlot(int pidSlotID, int closedLoopSlotID);

    void set_iZone(int pidSlotID, double iZone);

    void configAllowableErrorClosedLoop(int pidSlotID, double allowableError);

    void setMaxIAccumulation(int pidSlotID, double maxIAccum);

    void setPeakOutputClosedLoop(int pidSlotID, double peakOutput);

    void setIAccumulation(int closedLoopSlotID, double IAccum);

    double getClosedLoopError();

    double getIAccum(int closedLoopSlotID);

    double getErrorDerivative(int closedLoopSlotID);

    void setMotionProfileMaxVelocity(double maxVelocity);

    void setMotionProfileMaxAcceleration(double maxAcceleration);

    void configMotionCurve(MotionCurveType motionCurveType, int curveStrength);

    void clearMotionProfileTrajectoryBuffer();

    //Why are these Strings? Because I refuse to write a translator method with 50ish cases. Just no.
    String get_LastError();

    String get_Faults();

    String get_StickyFaults();

    int getFirmwareVersion();

    boolean hasResetOccurred();

    int getDeviceID();

    GreenControlMode get_ControlMode();

    void follow(IGreenMotorDev leader);

    double getSupplyCurrent();

    void restore_FactoryDefaults();

    boolean isVoltageCompensationEnabled();

    int getQuadraturePosition();

    void setQuadraturePosition(int quadraturePosition);

    int getPulseWidthPosition();

    boolean isFollower();

    SoftLimitStatus getSoftLimitStatus();

    void configControlFramePeriod(ControlFrame controlFrame, int periodms);

    /**
     * Updates the tracked enabled/disabled status of a motor
     *
     * @see SoftLimitStatus
     * @param softLimitStatus The current status of the motor
     * @param statusDemand The status demand applied
     */
    default SoftLimitStatus updateSoftLimitStatus(SoftLimitStatus softLimitStatus, SoftLimitStatus statusDemand) {
        switch(statusDemand){
            case FORWARD_DISABLE -> {
                if (softLimitStatus == SoftLimitStatus.FORWARD) {
                    softLimitStatus = SoftLimitStatus.DISABLED;
                } else if (softLimitStatus == SoftLimitStatus.BOTH) {
                    softLimitStatus = SoftLimitStatus.REVERSE;
                }
            }
            case REVERSE_DISABLE -> {
                if (softLimitStatus == SoftLimitStatus.REVERSE) {
                    softLimitStatus = SoftLimitStatus.DISABLED;
                } else if (softLimitStatus == SoftLimitStatus.BOTH) {
                    softLimitStatus = SoftLimitStatus.FORWARD;
                }
            }
            case FORWARD -> {
                if (softLimitStatus == SoftLimitStatus.REVERSE || softLimitStatus == SoftLimitStatus.BOTH) {
                    softLimitStatus = SoftLimitStatus.BOTH;
                } else {
                    softLimitStatus = SoftLimitStatus.FORWARD;
                }
            }
            case REVERSE -> {
                if (softLimitStatus == SoftLimitStatus.FORWARD || softLimitStatus == SoftLimitStatus.BOTH) {
                    softLimitStatus = SoftLimitStatus.BOTH;
                } else {
                    softLimitStatus = SoftLimitStatus.REVERSE;
                }
            }
            case DISABLED -> softLimitStatus = SoftLimitStatus.DISABLED;
        }
        return softLimitStatus;
    }

    enum MotorType {
        TALONFX,
        TALONSRX,
        VICTORSPX,
        SPARKMAX,
        GHOST
    }

}
