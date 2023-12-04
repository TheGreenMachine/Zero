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

    /**
     * Gets the name of a motor
     *
     * @return The motor name
     */
    String getName();

    /**
     * Gets the type of motor controller
     *
     * @see MotorType
     * @return The motor controller type
     */
    MotorType get_MotorType();

    /**
     * Gets a motor controller's firmware version
     * @return The firmware version
     */
    int getFirmwareVersion();

    /**
     * Gets a motor controller's CAN id
     * @return The CAN id
     */
    int getDeviceID();

    /** Active Motor Information */
    //From motor

    /**
     * Gets the current outputted by a motor controller
     * @return The output current (in amps)
     */
    double getOutputCurrent();

    /**
     * Gets the percent of motor power outputted by a motor controller
     * @return The percent out
     */
    double getMotorOutputPercent();

    /**
     * Gets the voltage outputted by a motor controller
     * @return The output voltage
     */
    double getMotorOutputVoltage();

    /**
     * Gets if a motor controller is inverted
     * @return If the controller is inverted
     */
    boolean getInverted();

    /**
     * Gets the temperature of the motor controller
     * @return The temperature (in Celsius)
     */
    double getMotorTemperature();

    /**
     * Gets the voltage of the battery, as seen by the motor controller
     * @return The battery voltage
     */
    double getBusVoltage();

    /**
     * Gets if a device reset has occurred since the last call to the motor controller
     * @return If a reset has occurred
     */
    boolean hasResetOccurred();

    /**
     * Gets the current supplied to the controlled motor
     * @return The current supplied (in amps)
     */
    double getSupplyCurrent();

    /**
     * Gets if the selected limit switch is currently closed
     * @param direction Forward or reverse limit switch
     * @return If the selected switch is closed
     */
    boolean isLimitSwitchClosed(LimitSwitchDirection direction);


    //From encoder

    /**
     * Gets the position recorded by the selected feedback sensor
     * @param closedLoopSlotID Primary or auxiliary PID
     * @return The sensor position (in ticks)
     */
    double getSensorPosition(int closedLoopSlotID);

    /**
     * Gets the velocity recorded by the selected feedback sensor
     * @param closedLoopSlotID Primary or auxiliary PID
     * @return The motor velocity (in sensor units/100ms)
     */
    double getSensorVelocity(int closedLoopSlotID);

    /**
     * Gets the difference between the target and actual closed-loop sensor value
     * @return The error (in ticks for position, units/100ms for velocity)
     */
    double getClosedLoopError();

    /**
     * Gets the Integral Accumulator value
     * @param closedLoopSlotID Primary or Auxiliary PID
     * @return The iAccum value
     */
    double getIAccum(int closedLoopSlotID);

    /**
     * Gets the derivative of the closed-loop error
     * @param closedLoopSlotID Primary or Auxiliary PID
     * @return The derivative of the closed-loop error
     */
    double getErrorDerivative(int closedLoopSlotID);

    /**
     * Gets the position recorded by a quadrature encoder
     * @return The quadrature position
     */
    int getQuadraturePosition();

    /**
     * Gets the position recorded by a pulse width encoder
     * @return The pulse width position
     */
    int getPulseWidthPosition();

    //From us! Sometimes.

    /**
     * Gets the control mode currently selected by a motor controller
     * @see GreenControlMode
     * @return The current control mode
     */
    GreenControlMode get_ControlMode();

    /**
     * Gets if voltage compensation has been enabled on a motor
     * @return If voltage compensation is enabled
     */
    boolean isVoltageCompensationEnabled();

    /**
     * Gets if a motor is a follower of another motor
     * @return If the motor is a follower
     */
    boolean isFollower();

    /**
     * Gets a motor's soft limit status
     * @see SoftLimitStatus
     * @return The soft limit status
     */
    SoftLimitStatus getSoftLimitStatus();


    //Faults
        //Why are these Strings? Because I refuse to write a translator method with 50ish cases. Just no.

    /**
     * Gets the most recent fault recorded by a motor controller
     * @return The most recent fault
     */
    String get_LastError();

    /**
     * Gets the collection of faults recorded by a motor controller
     * @return The recorded faults
     */
    String get_Faults();

    /**
     * Gets the collection of sticky faults recorded by a motor controller
     * @return The sticky faults
     */
    String get_StickyFaults();


    /** Control */
    /**
     * Sets the desired output to a motor controller
     *
     * @param controlMode The desired control mode
     * @param demand The desired demand
     */
    void set(GreenControlMode controlMode, double demand);

    /**
     * Sets a motor's output to neutral
     */
    void neutralOutput();

    /**
     * Sets the position currently read by the encoder as the value passed in
     * @param sensorPosition The position to be passed in
     * @param closedLoopSlotID Primary or Auxiliary PID
     */
    void setSensorPosition(double sensorPosition, int closedLoopSlotID);

    /**
     * Sets the position currently read by the encoder as the value passed in
     * @param sensorPosition The position to be passed in
     * @param closedLoopSlotID Primary or Auxiliary PID
     * @param timeoutMs The CAN timeout time (in ms)
     */
    void setSensorPosition(double sensorPosition, int closedLoopSlotID, int timeoutMs);

    /**
     * Makes a motor follow another motor
     * @param leader The motor to follow
     */
    void follow(IGreenMotor leader);

    /**
     * Sets the quadrature position currently read by the encoder as the value passed in
     * @param quadraturePosition The quadrature position to be passed in
     */
    void setQuadraturePosition(int quadraturePosition);

    /** Configurations */
    // Current limits

    /**
     * Configures a motor's current limits
     * @param configuration The configurations to be applied
     */
    void configCurrentLimit(SupplyCurrentLimitConfiguration configuration);

    /**
     * Configures a motor's current limits
     * @param configuration The configurations to be applied
     * @param timeoutMs The CAN timeout (in ms)
     */
    void configCurrentLimit(SupplyCurrentLimitConfiguration configuration, int timeoutMs);

    /**
     * Configures a motor's current limits
     * @param current The current limit
     */
    void configCurrentLimit(int current);

    // Status frames

    /**
     * Configures a motor's status frame period
     * @param statusFrame The type of status frame
     * @param periodms The period for the frame (in ms)
     */
    void setPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame, int periodms);

    /**
     * Gets the periodic status frame of a motor
     * @param statusFrame The status frame type
     * @return The period of the frame (in ms)
     */
    int getPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame);

    // Limit Switches

    /**
     * Configures the forward limit switch of a motor
     * @param normallyOpen If the motor is normally open
     */
    void configForwardLimitSwitch(boolean normallyOpen);

    /**
     * Configures the reverse limit switch of a motor
     * @param normallyOpen If the motor is normally open
     */
    void configReverseLimitSwitch(boolean normallyOpen);

    /**
     * Sets the enabled status of a motor's limit switches
     * @param isEnabled If the limit switches are enabled
     */
    void enableLimitSwitches(boolean isEnabled);



    // Ramp rates

    /**
     * Configures the ramp rate of a motor in open loop
     * @param secondsNeutralToFull The time it takes to reach the full demand
     */
    void configOpenLoopRampRate(double secondsNeutralToFull);

    /**
     * Configures the ramp rate of a motor in open loop
     * @param secondsNeutralToFull The time it takes to reach the full demand
     * @param timeoutMs The CAN timeout (in ms)
     */
    void configOpenLoopRampRate(double secondsNeutralToFull, int timeoutMs);

    /**
     * Configures the ramp rate of a motor in closed loop
     * @param secondsNeutralToFull The time it takes to reach the full demand
     */
    void configClosedLoopRampRate(double secondsNeutralToFull);

    /**
     * Configures the ramp rate of a motor in closed loop
     * @param secondsNeutralToFull The time it takes to reach the full demand
     * @param timeoutMs The CAN timeout (in ms)
     */
    void configClosedLoopRampRate(double secondsNeutralToFull, int timeoutMs);

    // Peak Outputs

    /**
     * Configures the motor's peak percent output forwards
     * @param percentOut The peak percent output
     */
    void config_PeakOutputForward(double percentOut);

    /**
     * Configures the motor's peak percent output forwards
     * @param percentOut The peak percent output
     * @param timeoutMs The CAN timeout (in ms)
     */
    void config_PeakOutputForward(double percentOut, int timeoutMs);

    /**
     * Configures the motor's peak percent output reverse
     * @param percentOut The peak percent output
     */
    void config_PeakOutputReverse(double percentOut);

    /**
     * Configures the motor's peak percent output reverse
     * @param percentOut The peak percent output
     * @param timeoutMs The CAN timeout (in ms)
     */
    void config_PeakOutputReverse(double percentOut, int timeoutMs);

    // Voltage compensation

    /**
     * Configures a motor's voltage compensation settings
     * @param voltage The maximum voltage to apply when compensation is enabled
     */
    void configVoltageCompensation(double voltage);

    /**
     * Sets if voltage compensation is enabled for a motor
     * @param isEnabled If voltage compensation should be enabled
     */
    void enableVoltageCompensation(boolean isEnabled);

    // Soft limits

    /**
     * Configures a motor's forwards soft limit
     * @param forwardSoftLimit The soft limit
     */
    void configForwardSoftLimit(double forwardSoftLimit);

    /**
     * Configures a motor's forwards soft limit
     * @param forwardSoftLimit The soft limit
     * @param timeoutMs The CAN timeout (in ms)
     */
    void configForwardSoftLimit(double forwardSoftLimit, int timeoutMs);

    /**
     * Configures a motor's reverse soft limit
     * @param reverseSoftLimit The soft limit
     */
    void configReverseSoftLimit(double reverseSoftLimit);

    /**
     * Configures a motor's reverse soft limit
     * @param reverseSoftLimit The soft limit
     * @param timeoutMs The CAN timeout (in ms)
     */
    void configReverseSoftLimit(double reverseSoftLimit, int timeoutMs);

    /**
     * Sets if the forwards soft limit is enabled on a motor
     * @param isEnabled If the forwards soft limit should be enabled
     */
    void enableForwardSoftLimit(boolean isEnabled);

    /**
     * Sets if the forwards soft limit is enabled on a motor
     * @param isEnabled If the forwards soft limit should be enabled
     * @param timeoutMs The CAN timeout (in ms)
     */
    void enableForwardSoftLimit(boolean isEnabled, int timeoutMs);

    /**
     * Sets if the reverse soft limit is enabled on a motor
     * @param isEnabled If the reverse soft limit should be enabled
     */
    void enableReverseSoftLimit(boolean isEnabled);

    /**
     * Sets if the reverse soft limit is enabled on a motor
     * @param isEnabled If the reverse soft limit should be enabled
     * @param timeoutMs The CAN timeout (in ms)
     */
    void enableReverseSoftLimit(boolean isEnabled, int timeoutMs);

    /**
     * Sets if the soft limits are enabled on a motor
     * @param isEnabled If the soft limits should be enabled
     */
    void enableSoftLimits(boolean isEnabled);

    //Misc

    /**
     * Configures the feedback sensor connected to a motor controller
     * @param deviceType The type of feedback sensor
     * @see FeedbackDeviceType
     */
    void selectFeedbackSensor(FeedbackDeviceType deviceType);

    /**
     * Configures the velocity measurement period of a motor's encoder
     * @param periodms The time of the period (in ms)
     */
    void setVelocityMeasurementPeriod(int periodms);

    /**
     * Configures the behavior of the motor while neutral
     * @param neutralMode The neutral mode type
     */
    void setNeutralMode(NeutralMode neutralMode);

    /**
     * Sets if the sensor phase of a motor's encoder is inverted
     * @param isInverted If the sensor phase should be inverted
     */
    void setSensorPhase(boolean isInverted);

    /**
     * Sets if the motor controller is inverted
     * @param isInverted If the motor should be inverted
     */
    void setInverted(boolean isInverted);

    //TODO will be deleted with phoenix six
    void config_NeutralDeadband(double deadbandPercent);

    /**
     * Restores a motor's factory default settings
     * @param timeoutMs The CAN timeout (in ms)
     */
    void restore_FactoryDefaults(int timeoutMs);

    /**
     * Configures the control frame period of a motor
     * @param controlFrame The desired control frame
     * @param periodms The period of the control frame (in ms)
     * @see ControlFrame
     */
    void configControlFramePeriod(ControlFrame controlFrame, int periodms);

    /** PID, Motion Profiling, and other motion goodies */
    //PID

    /**
     * Configures the kP of a PID slot
     * @param pidSlotID The slot to configure
     * @param kP The desired kP
     */
    void set_kP(int pidSlotID, double kP);

    /**
     * Configures the kI of a PID slot
     * @param pidSlotID The slot to configure
     * @param kI The desired kI
     */
    void set_kI(int pidSlotID, double kI);

    /**
     * Configures the kD of a PID slot
     * @param pidSlotID The slot to configure
     * @param kD The desired kD
     */
    void set_kD(int pidSlotID, double kD);

    /**
     * Configures the kF of a PID slot
     * @param pidSlotID The slot to configure
     * @param kF The desired kF
     */
    void set_kF(int pidSlotID, double kF);

    /**
     * Configures the arbitrary feed-forward value of a motor
     * @param feedForward The feed-forward value
     */
    void setArbitraryFeedForward(double feedForward);

    /**
     * Selects which PID slot a motor is using
     * @param pidSlotID The slot id
     * @param closedLoopSlotID Primary or auxiliary PID
     */
    void selectPIDSlot(int pidSlotID, int closedLoopSlotID);

    /**
     * Configures the integral zone for a PID slot
     * @param pidSlotID The PID slot to configure
     * @param iZone The desired integral zone
     */
    void set_iZone(int pidSlotID, double iZone);

    /**
     * Configures the maximum integral accumulation for a PID slot
     * @param pidSlotID The PID slot to configure
     * @param maxIAccum The desired max iAccum
     */
    void setMaxIAccumulation(int pidSlotID, double maxIAccum);

    /**
     * Sets the integral accumulation of a motor controller
     * @param closedLoopSlotID Primary or Auxiliary PID
     * @param IAccum The desired integral accumulation
     */
    void setIAccumulation(int closedLoopSlotID, double IAccum);

    //Motion Miscellaneous

    /**
     * Configures the allowable error during closed-loop control for a PID slot
     * @param pidSlotID The PID slot to configure
     * @param allowableError The desired allowable error
     */
    void configAllowableErrorClosedLoop(int pidSlotID, double allowableError);

    /**
     * Configures the allowable error during closed-loop control for a PID slot
     * @param pidSlotID The PID slot to configure
     * @param allowableError The desired allowable error
     * @param timeoutMs The CAN timeout (in ms)
     */
    void configAllowableErrorClosedLoop(int pidSlotID, double allowableError, int timeoutMs);

    /**
     * Configures the peak output during closed-loop control for a PID slot
     * @param pidSlotID The PID slot to configure
     * @param peakOutput The desired peak output
     */
    void setPeakOutputClosedLoop(int pidSlotID, double peakOutput);

    /**
     * Configures the peak output during closed-loop control for a PID slot
     * @param pidSlotID The PID slot to configure
     * @param peakOutput The desired peak output
     * @param timeoutMs The CAN timeout (in ms)
     */
    void setPeakOutputClosedLoop(int pidSlotID, double peakOutput, int timeoutMs);

    //Motion Profiling

    /**
     * Configures the maximum velocity during motion-profiling
     * @param maxVelocity The desired maximum velocity
     */
    void setMotionProfileMaxVelocity(double maxVelocity);

    /**
     * Configures the maximum velocity during motion-profiling
     * @param maxVelocity The desired maximum velocity
     * @param timeoutMs The CAN timeout (in ms)
     */
    void setMotionProfileMaxVelocity(double maxVelocity, int timeoutMs);

    /**
     * Configures the maximum acceleration during motion-profiling
     * @param maxAcceleration The desired maximum acceleration
     */
    void setMotionProfileMaxAcceleration(double maxAcceleration);

    /**
     * Configures the maximum acceleration during motion-profiling
     * @param maxAcceleration The desired maximum acceleration
     * @param timeoutMs The CAN timeout (in ms)
     */
    void setMotionProfileMaxAcceleration(double maxAcceleration, int timeoutMs);

    /**
     * Configures the motion curve to use during motion profiling
     * @param motionCurveType The type of motion curve
     * @param curveStrength The strength of the curve
     */
    void configMotionCurve(MotionCurveType motionCurveType, int curveStrength);

    /**
     * Clears the trajectory buffer of a motion profile
     */
    void clearMotionProfileTrajectoryBuffer();

    /**
     * Updates the tracked status of a motor's soft limits
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

    enum LimitSwitchDirection {
        FORWARD,
        REVERSE
    }

}
