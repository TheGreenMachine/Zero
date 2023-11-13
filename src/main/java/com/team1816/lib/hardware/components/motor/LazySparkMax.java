package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.*;
import com.team1816.lib.hardware.components.motor.configurations.*;
import com.team1816.lib.util.ConfigurationTranslator;
import com.team1816.lib.util.logUtil.GreenLogger;

public class LazySparkMax extends CANSparkMax implements IGreenMotor {
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;

    protected String name = "";
    protected GreenControlMode currentControlMode = GreenControlMode.PERCENT_OUTPUT;
    protected SoftLimitStatus softLimitStatus;
    protected int currentPeriodicFrame = -1;
    protected int currentPIDSlot = 0;

    protected SparkMaxLimitSwitch forwardLimitSwitch, reverseLimitSwitch = null;

    protected double peakOutputForward, peakOutputBackward = -0;
    protected double nominalOutputForward, nominalOutputBackward = 0;

    protected double voltageForCompensation = 0;
    protected boolean voltageCompensationEnabled = false;
    protected double arbitraryFeedForward = 0;


    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceNumber The device ID.
     * @param motorName The name of the motor
     */
    public LazySparkMax(int deviceNumber, String motorName) {
        super(deviceNumber, CANSparkMaxLowLevel.MotorType.kBrushless);
        pidController = super.getPIDController();
        encoder = configureRelativeEncoder(FeedbackDeviceType.HALL_SENSOR);
        name = motorName;
        softLimitStatus = SoftLimitStatus.DISABLED;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public IGreenMotor.MotorType get_MotorType() {
        return IGreenMotor.MotorType.SparkMax;
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType) {
        encoder = configureRelativeEncoder(deviceType);
    }

    private RelativeEncoder configureRelativeEncoder(FeedbackDeviceType deviceType) {
        return super.getEncoder(
            ConfigurationTranslator.toSparkMaxRelativeEncoderType(deviceType),
            42
        );
    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration) {
        configCurrentLimit((int) configuration.currentLimit);
    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration, int timeoutMs) {
        configCurrentLimit(configuration);
    }

    @Override
    public void configCurrentLimit(int current) {
        super.setSmartCurrentLimit(current);
    }

    @Override
    public void setPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame, int periodms) {
        currentPeriodicFrame = periodms;
        super.setPeriodicFramePeriod(
            ConfigurationTranslator.toPeriodicFrame(statusFrame),
            periodms
        );
    }

    @Override
    public int getPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame) {
        return currentPeriodicFrame;
    }

    @Override
    public void setVelocityMeasurementPeriod(int periodms) {
        encoder.setMeasurementPeriod(periodms);
    }

    @Override
    public void set(GreenControlMode controlMode, double demand) {
        if (demand > 0 && demand < nominalOutputForward) {
            demand = nominalOutputForward;
        } else if (demand < 0 && demand > -nominalOutputBackward) {
            demand = -nominalOutputBackward;
        }
        currentControlMode = controlMode;
        pidController.setReference(
            demand,
            ConfigurationTranslator.toSparkMaxControlType(controlMode),
            currentPIDSlot,
            arbitraryFeedForward, //Note that arbitraryFF is initialized to 0
            SparkMaxPIDController.ArbFFUnits.kPercentOut
        );
    }

    @Override
    public void configForwardLimitSwitch(boolean normallyOpen) {
        forwardLimitSwitch = super.getForwardLimitSwitch(normallyOpen ? SparkMaxLimitSwitch.Type.kNormallyOpen : SparkMaxLimitSwitch.Type.kNormallyClosed);
    }

    @Override
    public void configReverseLimitSwitch(boolean normallyOpen) {
        reverseLimitSwitch = super.getReverseLimitSwitch(normallyOpen ? SparkMaxLimitSwitch.Type.kNormallyOpen : SparkMaxLimitSwitch.Type.kNormallyClosed);
    }

    @Override
    public void neutralOutput() {
        super.stopMotor();
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        super.setIdleMode(neutralMode == NeutralMode.Brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void setSensorPhase(boolean isInverted) {
        encoder.setInverted(isInverted); // This is NOT the same as a call to super.getInverted().
    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull) {
        super.setOpenLoopRampRate(secondsNeutralToFull);
    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull, int timeoutMs) {
        configOpenLoopRampRate(secondsNeutralToFull);
    }

    @Override
    public void configClosedLoopRampRate(double secondsNeutralToFull) {
        super.setClosedLoopRampRate(secondsNeutralToFull);
    }

    @Override
    public void configClosedLoopRampRate(double secondsNeutralToFull, int timeoutMs) {
        configClosedLoopRampRate(secondsNeutralToFull);
    }

    @Override
    public void config_PeakOutputForward(double percentOut) {
        peakOutputForward = percentOut;
        pidController.setOutputRange(peakOutputBackward, peakOutputForward, currentPIDSlot);
    }

    @Override
    public void config_PeakOutputForward(double percentOut, int timeoutMs) {
        config_PeakOutputForward(percentOut);
    }


    @Override
    public void config_PeakOutputReverse(double percentOut) {
        //Use negative values for backwards range
        peakOutputBackward = percentOut;
        pidController.setOutputRange(peakOutputBackward, peakOutputForward, currentPIDSlot);
    }

    @Override
    public void config_PeakOutputReverse(double percentOut, int timeoutMs) {
        config_PeakOutputReverse(percentOut);
    }


    @Override
    public void config_NominalOutputForward(double percentOut) {
        nominalOutputForward = percentOut;
    }

    @Override
    public void config_NominalOutputForward(double percentOut, int timeoutMs) {
        config_NominalOutputForward(percentOut);
    }

    @Override
    public void config_NominalOutputReverse(double percentOut) {
        nominalOutputBackward = percentOut;
    }

    @Override
    public void config_NominalOutputReverse(double percentOut, int timeoutMs) {
        config_NominalOutputReverse(percentOut);
    }


    /**
     * @see <a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces">Documentation</a>
     */
    @Override
    public void config_NeutralDeadband(double deadbandPercent) {
        GreenLogger.log("Neutral deadband is only configurable through USB for Spark Max. Factory default is ±5%");

    }

    @Override
    public void configVoltageCompensation(double voltage) {
        voltageForCompensation = voltage;
    }

    @Override
    public void enableVoltageCompensation(boolean isEnabled) {
        if (isEnabled) {
            super.enableVoltageCompensation(voltageForCompensation);
            voltageCompensationEnabled = true;
        } else {
            super.disableVoltageCompensation();
            voltageCompensationEnabled = false;
        }
    }

    @Override
    public double getMotorOutputPercent() {
        return super.getAppliedOutput(); // We don't use get() because that is only supplied with set() and we skip over that for setReference()
    }

    @Override
    public double getMotorOutputVoltage() {
        return getMotorOutputPercent() * getBusVoltage(); //hate this but it's literally how BaseMotorController does it
    }

    @Override
    public double getSensorPosition(int closedLoopSlotID) {
        return encoder.getPosition();
    }

    @Override
    public double getSensorVelocity(int closedLoopSlotID) {
        return encoder.getVelocity();
    }

    @Override
    public void setSensorPosition(double sensorPosition, int closedLoopSlotID) {
        encoder.setPosition(sensorPosition);
    }

    @Override
    public void setSensorPosition(double sensorPosition, int closedLoopSlotID, int timeoutMs) {
        setSensorPosition(sensorPosition, closedLoopSlotID);
    }

    @Override
    public void enableLimitSwitches(boolean isEnabled) {
        //WHY DO LIMIT SWITCHES HAVE A TOGGLE PARAMETER BUT VOLTAGE COMPENSATION DOESNT
        if (forwardLimitSwitch == null || reverseLimitSwitch == null) {
            configForwardLimitSwitch(true);
            configReverseLimitSwitch(true);
        }

        forwardLimitSwitch.enableLimitSwitch(isEnabled);
        reverseLimitSwitch.enableLimitSwitch(isEnabled);
    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit) {
        //free me from this torture. why does this use a float when every other rev method uses doubles?
        super.setSoftLimit(SoftLimitDirection.kForward, (float) forwardSoftLimit);
    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit, int timeoutMs) {
        configForwardSoftLimit(forwardSoftLimit);
    }


    @Override
    public void configReverseSoftLimit(double reverseSoftLimit) {
        super.setSoftLimit(SoftLimitDirection.kReverse, (float) reverseSoftLimit);
    }

    @Override
    public void configReverseSoftLimit(double reverseSoftLimit, int timeoutMs) {
        configReverseSoftLimit(reverseSoftLimit);
    }

    @Override
    public void enableForwardSoftLimit(boolean isEnabled) {
        super.enableSoftLimit(SoftLimitDirection.kForward, isEnabled);
        softLimitStatus = updateSoftLimitStatus(
            softLimitStatus,
            isEnabled ? SoftLimitStatus.FORWARD : SoftLimitStatus.FORWARD_DISABLE
        );
    }

    @Override
    public void enableForwardSoftLimit(boolean isEnabled, int timeoutMs) {
        enableForwardSoftLimit(isEnabled);
    }


    @Override
    public void enableReverseSoftLimit(boolean isEnabled) {
        super.enableSoftLimit(SoftLimitDirection.kReverse, isEnabled);
        softLimitStatus = updateSoftLimitStatus(
            softLimitStatus,
            isEnabled ? SoftLimitStatus.REVERSE : SoftLimitStatus.REVERSE_DISABLE
        );
    }

    @Override
    public void enableSoftLimits(boolean isEnabled) {
        super.enableSoftLimit(SoftLimitDirection.kForward, isEnabled);
        super.enableSoftLimit(SoftLimitDirection.kReverse, isEnabled);
        softLimitStatus = updateSoftLimitStatus(
            softLimitStatus,
            isEnabled ? SoftLimitStatus.BOTH : SoftLimitStatus.DISABLED
        );
    }

    @Override
    public void enableReverseSoftLimit(boolean isEnabled, int timeoutMs) {
        enableReverseSoftLimit(isEnabled);
    }


    @Override
    public void set_kP(int pidSlotID, double kP) {
        pidController.setP(kP, pidSlotID);
    }

    @Override
    public void set_kI(int pidSlotID, double kI) {
        pidController.setI(kI, pidSlotID);
    }

    @Override
    public void set_kD(int pidSlotID, double kD) {
        pidController.setD(kD, pidSlotID);
    }

    @Override
    public void set_kF(int pidSlotID, double kF) {
        pidController.setFF(kF, pidSlotID);
    }

    @Override
    public void setArbitraryFeedForward( double feedForward) {
        arbitraryFeedForward = feedForward;
    }

    @Override
    public double getFeedForward(int closedLoopSlotID, int PIDSlotID) {
        return pidController.getFF(PIDSlotID);
    }

    @Override
    public void selectPIDSlot(int pidSlotID, int closedLoopSlotID) {
        currentPIDSlot = pidSlotID;
    }

    @Override
    public void set_iZone(int pidSlotID, double iZone) {
        pidController.setIZone(iZone, pidSlotID);
    }

    @Override
    public void configAllowableErrorClosedLoop(int pidSlotID, double allowableError) {
        GreenLogger.log("Allowable error is only configurable through USB for Spark Max.");
    }

    @Override
    public void configAllowableErrorClosedLoop(int pidSlotID, double allowableError, int timeoutMs) {
        configAllowableErrorClosedLoop(pidSlotID, allowableError);
    }


    @Override
    public void setMaxIAccumulation(int pidSlotID, double maxIAccum) {
        pidController.setIMaxAccum(maxIAccum, pidSlotID);
    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput) {
        pidController.setOutputRange(-peakOutput, peakOutput, pidSlotID);
    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput, int timeoutMs) {
        setPeakOutputClosedLoop(pidSlotID, peakOutput);
    }

    @Override
    public void setIAccumulation(int closedLoopSlotID, double IAccum) {
        pidController.setIAccum(IAccum);
    }

    @Override
    public double getClosedLoopError() {
        //This is theoretically possible in a few ways
            // The actual firmware implementation is here https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control but error is not retrievable
                //if we figured out what pv meant then we could calc it ourselves
            // Could also reverse engineer the output from the PID equation but that could potentially be really slow
        return Double.NaN;
    }

    @Override
    public double getIAccum(int closedLoopSlotID) {
        return pidController.getIAccum();
    }

    @Override
    public double getErrorDerivative(int closedLoopSlotID) {
        // Literally just dy/dx error for each loop iteration (I HAD TO LOOK THROUGH 5 YEAR OLD DOCUMENTATION TO FIGURE OUT THAT ITS SO SIMPLE THANK YOU UNCLEAR METHOD NAMES)
        // Easy to do Math wise but have urva do anyways cause she wants to learn more CS
        GreenLogger.log("Error Derivative unavailable for Spark Max Motors +C");
        return Double.NaN;
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity) {
        pidController.setSmartMotionMaxVelocity(maxVelocity, currentPIDSlot);
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity, int timeoutMs) {
        setMotionProfileMaxVelocity(maxVelocity);
    }


    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration) {
        pidController.setSmartMotionMaxAccel(maxAcceleration, currentPIDSlot);
    }

    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration, int timeoutMs) {
        setMotionProfileMaxAcceleration(maxAcceleration);
    }

    @Override
    public void configMotionCurve(MotionCurveType motionCurveType, int curveStrength) {
        pidController.setSmartMotionAccelStrategy(
            motionCurveType == MotionCurveType.S_CURVE ? SparkMaxPIDController.AccelStrategy.kSCurve : SparkMaxPIDController.AccelStrategy.kTrapezoidal,
            currentPIDSlot
        );
    }

    @Override
    public void clearMotionProfileTrajectoryBuffer() {
        GreenLogger.log("Cannot clear motion profile trajectories on SparkMax.");
    }

    @Override
    public String get_LastError() {
        return "REVLibErrorCode: " + super.getLastError().name();
    }

    @Override
    //Would it even be useful to test this and see if it corresponds with FaultID.value()? is it even possible?
    //I'm so sorry to anyone looking at this code if it ever causes problems.
    public String get_Faults() {
        return "All faults for " + getName() + " as a short: " + super.getFaults();
    }

    @Override
    public String get_StickyFaults() {
        return "All sticky faults for " + getName() + " as a short: " + super.getStickyFaults();
    }

    @Override
    public boolean hasResetOccurred() {
        GreenLogger.log("SparkMax does not track resets.");
        return false;
    }

    @Override
    public int getDeviceID() {
        return super.getDeviceId();
    }

    @Override
    public GreenControlMode get_ControlMode() {
        return currentControlMode;
    }

    @Override
    public void follow(IGreenMotor leader) {
        if (leader instanceof LazySparkMax) {
            super.follow((CANSparkMax) leader);
        } else {
            super.follow(ExternalFollower.kFollowerPhoenix, leader.getDeviceID());
        }
    }

    @Override
    public double getSupplyCurrent() {
        return getOutputCurrent();
    }

    @Override
    public void restore_FactoryDefaults(int timeoutMs) {
        super.restoreFactoryDefaults();
    }

    @Override
    public boolean isVoltageCompensationEnabled() {
        return voltageCompensationEnabled;
    }

    @Override
    public int getQuadraturePosition() {
        return encoder.getAverageDepth(); //I think. Check w DDAY
    }

    @Override
    public void setQuadraturePosition(int quadraturePosition) {
        encoder.setAverageDepth(quadraturePosition); //I think. Check w DDAY
    }

    @Override
    public int getPulseWidthPosition() {
        //I Think this could be done with some math? ask DDAY
        return -1;
    }

    @Override
    public SoftLimitStatus getSoftLimitStatus() {
        return softLimitStatus;
    }

    @Override
    public void configControlFramePeriod(ControlFrame controlFrame, int periodms) {
        super.setControlFramePeriodMs(periodms);
    }
}
