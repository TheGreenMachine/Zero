package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.team1816.lib.hardware.components.motor.configurations.*;

public interface IGreenMotor {
    //TODO Annotate
    //TODO sort child class methods

    //Some getter methods add an underscore after get to avoid conflicts in LazyMotor classes

    /** Static Motor Information */
    String getName();

    MotorType get_MotorType();

    int getFirmwareVersion();

    int getDeviceID();

    /** Active Motor Information */
    //From motor
    double getOutputCurrent();

    boolean getInverted();

    double getMotorTemperature();

    double getBusVoltage();

    boolean hasResetOccurred();

    double getSupplyCurrent();

    //From encoder
    double getMotorOutputPercent();

    double getMotorOutputVoltage();

    double getSensorPosition(int closedLoopSlotID);

    double getSensorVelocity(int closedLoopSlotID);

    double getClosedLoopError();

    double getIAccum(int closedLoopSlotID);

    double getErrorDerivative(int closedLoopSlotID);

    int getQuadraturePosition();

    int getPulseWidthPosition();

    //From us! Sometimes.
    GreenControlMode get_ControlMode();

    boolean isVoltageCompensationEnabled();

    boolean isFollower();

    SoftLimitStatus getSoftLimitStatus();


    //Faults
        //Why are these Strings? Because I refuse to write a translator method with 50ish cases. Just no.
    String get_LastError();

    String get_Faults();

    String get_StickyFaults();


    /** Control */
    void set(GreenControlMode controlMode, double demand);

    void neutralOutput();

    void setSensorPosition(double sensorPosition, int closedLoopSlotID);

    void setSensorPosition(double sensorPosition, int closedLoopSlotID, int timeoutMs);

    void follow(IGreenMotor leader);

    void setQuadraturePosition(int quadraturePosition);

    /** Configurations */
    // Current limits
    void configCurrentLimit(SupplyCurrentLimitConfiguration configuration);

    void configCurrentLimit(SupplyCurrentLimitConfiguration configuration, int timeoutMs);

    void configCurrentLimit(int current);

    // Status frames
    void setPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame, int periodms);

    int getPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame);

    // Limit Switches
    void configForwardLimitSwitch(boolean normallyOpen);

    void configReverseLimitSwitch(boolean normallyOpen);

    void enableLimitSwitches(boolean isEnabled);


    // Ramp rates
    void configOpenLoopRampRate(double secondsNeutralToFull);

    void configOpenLoopRampRate(double secondsNeutralToFull, int timeoutMs);

    void configClosedLoopRampRate(double secondsNeutralToFull);

    void configClosedLoopRampRate(double secondsNeutralToFull, int timeoutMs);

    // Peak Outputs
    void config_PeakOutputForward(double percentOut);

    void config_PeakOutputForward(double percentOut, int timeoutMs);

    void config_PeakOutputReverse(double percentOut);

    void config_PeakOutputReverse(double percentOut, int timeoutMs);

    // Voltage compensation
    void configVoltageCompensation(double voltage);

    void enableVoltageCompensation(boolean isEnabled);

    // Soft limits
    void configForwardSoftLimit(double forwardSoftLimit);

    void configForwardSoftLimit(double forwardSoftLimit, int timeoutMs);

    void configReverseSoftLimit(double reverseSoftLimit);

    void configReverseSoftLimit(double reverseSoftLimit, int timeoutMs);

    void enableForwardSoftLimit(boolean isEnabled);

    void enableForwardSoftLimit(boolean isEnabled, int timeoutMs);

    void enableReverseSoftLimit(boolean isEnabled);

    void enableReverseSoftLimit(boolean isEnabled, int timeoutMs);

    void enableSoftLimits(boolean isEnabled);

    //Misc
    void selectFeedbackSensor(FeedbackDeviceType deviceType);

    void setVelocityMeasurementPeriod(int periodms);

    void setNeutralMode(NeutralMode neutralMode);

    void setSensorPhase(boolean isInverted);

    void setInverted(boolean isInverted);

    void config_NeutralDeadband(double deadbandPercent);

    void restore_FactoryDefaults(int timeoutMs);

    void configControlFramePeriod(ControlFrame controlFrame, int periodms);

    /** PID, Motion Profiling, and other motion goodies */
    //PID
    void set_kP(int pidSlotID, double kP);

    void set_kI(int pidSlotID, double kI);

    void set_kD(int pidSlotID, double kD);

    void set_kF(int pidSlotID, double kF);

    void setArbitraryFeedForward(double feedForward);

    double getFeedForward(int closedLoopSlotID, int pidSlotID);

    void selectPIDSlot(int pidSlotID, int closedLoopSlotID);

    void set_iZone(int pidSlotID, double iZone);

    void setMaxIAccumulation(int pidSlotID, double maxIAccum);

    void setIAccumulation(int closedLoopSlotID, double IAccum);
    //Motion Miscellaneous
    void configAllowableErrorClosedLoop(int pidSlotID, double allowableError);

    void configAllowableErrorClosedLoop(int pidSlotID, double allowableError, int timeoutMs);

    void setPeakOutputClosedLoop(int pidSlotID, double peakOutput);

    void setPeakOutputClosedLoop(int pidSlotID, double peakOutput, int timeoutMs);

    //Motion Profiling
    void setMotionProfileMaxVelocity(double maxVelocity);

    void setMotionProfileMaxVelocity(double maxVelocity, int timeoutMs);

    void setMotionProfileMaxAcceleration(double maxAcceleration);

    void setMotionProfileMaxAcceleration(double maxAcceleration, int timeoutMs);

    void configMotionCurve(MotionCurveType motionCurveType, int curveStrength);

    void clearMotionProfileTrajectoryBuffer();

    /**
     * Updates the tracked enabled/disabled status of a motor's soft limits
     *
     * @see SoftLimitStatus
     * @param softLimitStatus The current soft limit status of the motor
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
        TalonFX, //Falcons and Krakens
        TalonSRX, //cims, bags, etc
        VictorSPX, //no idea what this actually controls, if we ever use these again the world is probably ending
        SparkMax, //neo 550s
        GHOST //simulation
    }

}
