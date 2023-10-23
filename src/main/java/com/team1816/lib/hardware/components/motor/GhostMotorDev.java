package com.team1816.lib.hardware.components.motor;

import com.ctre.phoenix.motorcontrol.*;
import com.team1816.lib.hardware.components.motor.configurations.*;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import static java.lang.Double.NaN;

public class GhostMotorDev implements IGreenMotor {
    protected String name = "";

    /**
     * Characterization
     */
    private final int maxVelTicks100ms;
    private final int absInitOffset;
    private int fwdLimit;
    private int revLimit;
    private boolean usingLimit = false;
    private final int absMotorPPR = 4096;
    /**
     * State
     */
    private GreenControlMode controlMode;
    private final double[] desiredDemand = new double[]{0, 0, 0}; // 0: %out, 1: vel, 2: pos, 3: Motion magic
    private final double[] actualOutput = new double[]{0, 0, 0}; // 0: %out, 1: vel, 2: pos, 3: Motion Magic
    protected double lastPos = 0;

    private double motionMagicCruiseVel;
    private double motionMagicAccel;

    protected double lastUpdate = 0;

    protected SoftLimitStatus softLimitStatus = SoftLimitStatus.DISABLED;

    public GhostMotorDev(int maxTickVel, int absInitOffset, String motorName) {
        this.absInitOffset = absInitOffset;
        maxVelTicks100ms = maxTickVel;
        name = motorName;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public MotorType get_MotorType() {
        return MotorType.GHOST;
    }

    @Override
    public void selectFeedbackSensor(FeedbackDeviceType deviceType) {

    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration) {

    }

    @Override
    public void configCurrentLimit(SupplyCurrentLimitConfiguration configuration, int timeoutMs) {

    }


    @Override
    public void configCurrentLimit(int current) {

    }

    @Override
    public void setPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame, int periodms) {

    }

    @Override
    public int getPeriodicStatusFramePeriod(PeriodicStatusFrame statusFrame) {
        return 0;
    }

    @Override
    public double getOutputCurrent() {
        return 0;
    }

    @Override
    public void setVelocityMeasurementPeriod(int periodms) {

    }

    @Override
    public void set(GreenControlMode Mode, double demand) {
        processSet(controlMode, demand);
    }

    private void processSet(GreenControlMode controlModeDemand, double demand) {
        // setting desired demand
        if (controlModeDemand == GreenControlMode.PERCENT_OUTPUT) {
            desiredDemand[0] = demand;
            desiredDemand[1] = NaN;
            desiredDemand[2] = NaN;
        } else if (controlModeDemand == GreenControlMode.VELOCITY_CONTROL) {
            this.desiredDemand[0] = NaN;
            this.desiredDemand[1] = demand;
            this.desiredDemand[2] = NaN;
        } else if (controlModeDemand == GreenControlMode.POSITION_CONTROL || controlModeDemand == GreenControlMode.MOTION_MAGIC) {
            this.desiredDemand[0] = NaN;
            this.desiredDemand[1] = NaN;
            this.desiredDemand[2] = demand;
        } else {
            GreenLogger.log("no support for this Mode in GhostMotor!");
            return;
        }
        controlMode = controlModeDemand;
    }

    private void updateActValues() {
        // don't make unnecessary calculations if robot not in sim
        if (RobotBase.isReal()) {
            return;
        }

        // whether motor needs to calculate new numbers - this
        double timeNow = Timer.getFPGATimestamp();
        double dtBetweenCallsMS = (timeNow - lastUpdate) * 1000;
        if (dtBetweenCallsMS < Robot.robotDt * 0.75) {
            lastUpdate = timeNow;
            return;
        }
        lastUpdate = timeNow;

        // setting actual output
        if (controlMode == GreenControlMode.PERCENT_OUTPUT) {
            actualOutput[0] = desiredDemand[0];
            actualOutput[1] = desiredDemand[0] * maxVelTicks100ms;
            actualOutput[2] = lastPos + (actualOutput[1] / 100 * dtBetweenCallsMS);
        } else if (controlMode == GreenControlMode.VELOCITY_CONTROL) {
            actualOutput[0] = desiredDemand[1] / maxVelTicks100ms;
            actualOutput[1] = desiredDemand[1];
            actualOutput[2] = lastPos + (actualOutput[1] / 100 * dtBetweenCallsMS);
        } else if (controlMode == GreenControlMode.POSITION_CONTROL) {
            double desaturatedVel = Math.signum(desiredDemand[2] - lastPos) * Math.min(maxVelTicks100ms, Math.abs(desiredDemand[2] - lastPos) / dtBetweenCallsMS * 100);

            actualOutput[0] = desaturatedVel / maxVelTicks100ms;
            actualOutput[1] = desaturatedVel;
            actualOutput[2] = lastPos + (actualOutput[1] / 100 * dtBetweenCallsMS);
        } else if (controlMode == GreenControlMode.MOTION_MAGIC) {
            // not accounting for accel rn - just using motionMagicCruiseVel
            double accelAccountedVel = Math.min(motionMagicCruiseVel, Math.abs(actualOutput[1]) + (motionMagicAccel / 100 * dtBetweenCallsMS));
            double desaturatedVel = Math.signum(desiredDemand[2] - lastPos) * Math.min(accelAccountedVel, Math.abs(desiredDemand[2] - lastPos) / dtBetweenCallsMS * 100);
            actualOutput[0] = desaturatedVel / maxVelTicks100ms;
            actualOutput[1] = desaturatedVel;
            actualOutput[2] = lastPos + (actualOutput[1] / 100 * dtBetweenCallsMS);
        }

        if (usingLimit) {
            if (actualOutput[2] >= fwdLimit) {
                actualOutput[0] = 0;
                actualOutput[1] = 0;
                actualOutput[2] = fwdLimit;
            } else if (actualOutput[2] <= revLimit) {
                actualOutput[0] = 0;
                actualOutput[1] = 0;
                actualOutput[2] = revLimit;
            }
        }

        lastPos = actualOutput[2];
    }

    @Override
    public void configForwardLimitSwitch(boolean normallyOpen) {

    }

    @Override
    public void configReverseLimitSwitch(boolean normallyOpen) {

    }

    @Override
    public void neutralOutput() {

    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {

    }

    @Override
    public void setSensorPhase(boolean isInverted) {

    }

    @Override
    public void setInverted(boolean isInverted) {

    }

    @Override
    public boolean getInverted() {
        return false;
    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull) {

    }

    @Override
    public void configOpenLoopRampRate(double secondsNeutralToFull, int timeoutMs) {

    }


    @Override
    public void configClosedLoopRampRate(double secondsNeutralToFull) {

    }

    @Override
    public void configClosedLoopRampRate(double secondsNeutralToFull, int timeoutMs) {

    }

    @Override
    public void config_PeakOutputForward(double percentOut) {

    }

    @Override
    public void config_PeakOutputForward(double percentOut, int timeoutMs) {

    }


    @Override
    public void config_PeakOutputReverse(double percentOut) {

    }

    @Override
    public void config_PeakOutputReverse(double percentOut, int timeoutMs) {

    }

    @Override
    public void config_NominalOutputForward(double percentOut) {

    }

    @Override
    public void config_NominalOutputForward(double percentOut, int timeoutMs) {

    }

    @Override
    public void config_NominalOutputReverse(double percentOut) {

    }

    @Override
    public void config_NominalOutputReverse(double percentOut, int timeoutMs) {

    }

    @Override
    public void config_NeutralDeadband(double deadbandPercent) {

    }

    @Override
    public void configVoltageCompensation(double voltage) {

    }

    @Override
    public void enableVoltageCompensation(boolean isEnabled) {

    }

    @Override
    public double getBusVoltage() {
        return 12;
    }

    @Override
    public double getMotorOutputPercent() {
        updateActValues();
        return actualOutput[0];
    }

    @Override
    public double getMotorOutputVoltage() {
        return getMotorOutputPercent() * RobotController.getBatteryVoltage();
    }

    @Override
    public double getMotorTemperature() {
        return 0;
    }

    @Override
    public double getSensorPosition(int closedLoopSlotID) {
        updateActValues();
        return actualOutput[2];
    }

    @Override
    public double getSensorVelocity(int closedLoopSlotID) {
        updateActValues();
        return actualOutput[1];
    }

    @Override
    public void setSensorPosition(double sensorPosition, int closedLoopSlotID) {
        processSet(GreenControlMode.POSITION_CONTROL, sensorPosition);
    }

    @Override
    public void setSensorPosition(double sensorPosition, int closedLoopSlotID, int timeoutMs) {
        setSensorPosition(sensorPosition, closedLoopSlotID);
    }


    @Override
    public void enableLimitSwitches(boolean isEnabled) {

    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit) {
        usingLimit = true;
        fwdLimit = (int) forwardSoftLimit;
    }

    @Override
    public void configForwardSoftLimit(double forwardSoftLimit, int timeoutMs) {
        configForwardSoftLimit(forwardSoftLimit);
    }


    @Override
    public void configReverseSoftLimit(double reverseSoftLimit) {
        usingLimit = true;
        revLimit = (int) reverseSoftLimit;
    }

    @Override
    public void configReverseSoftLimit(double reverseSoftLimit, int timeoutMs) {
        configReverseSoftLimit(reverseSoftLimit);
    }


    @Override
    public void enableForwardSoftLimit(boolean isEnabled) {
        usingLimit = isEnabled;
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
        usingLimit = isEnabled;
        softLimitStatus = updateSoftLimitStatus(
            softLimitStatus,
            isEnabled ? SoftLimitStatus.REVERSE : SoftLimitStatus.REVERSE_DISABLE
        );
    }

    @Override
    public void enableReverseSoftLimit(boolean isEnabled, int timeoutMs) {
        enableReverseSoftLimit(isEnabled);
    }

    @Override
    public void enableSoftLimits(boolean isEnabled) {
        usingLimit = isEnabled;
        softLimitStatus = updateSoftLimitStatus(
            softLimitStatus,
            isEnabled ? SoftLimitStatus.BOTH : SoftLimitStatus.DISABLED
        );
    }

    @Override
    public void set_kP(int pidSlotID, double kP) {

    }

    @Override
    public void set_kI(int pidSlotID, double kI) {

    }

    @Override
    public void set_kD(int pidSlotID, double kD) {

    }

    @Override
    public void set_kF(int pidSlotID, double kF) {

    }

    @Override
    public void setArbitraryFeedForward(double feedForward) {

    }

    @Override
    public double getFeedForward(int closedLoopSlotID, int pidSlotID) {
        return 0;
    }

    @Override
    public void selectPIDSlot(int pidSlotID, int closedLoopSlotID) {

    }

    @Override
    public void set_iZone(int pidSlotID, double iZone) {

    }

    @Override
    public void configAllowableErrorClosedLoop(int pidSlotID, double allowableError) {

    }

    @Override
    public void configAllowableErrorClosedLoop(int pidSlotID, double allowableError, int timeoutMs) {

    }


    @Override
    public void setMaxIAccumulation(int pidSlotID, double maxIAccum) {

    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput) {

    }

    @Override
    public void setPeakOutputClosedLoop(int pidSlotID, double peakOutput, int timeoutMs) {

    }


    @Override
    public void setIAccumulation(int closedLoopSlotID, double IAccum) {

    }

    @Override
    public double getClosedLoopError() {
        return 0;
    }

    @Override
    public double getIAccum(int closedLoopSlotID) {
        return 0;
    }

    @Override
    public double getErrorDerivative(int closedLoopSlotID) {
        return 0;
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity) {
        motionMagicCruiseVel = maxVelocity;
    }

    @Override
    public void setMotionProfileMaxVelocity(double maxVelocity, int timeoutMs) {
        setMotionProfileMaxVelocity(maxVelocity);
    }


    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration) {
        motionMagicAccel = maxAcceleration;
    }

    @Override
    public void setMotionProfileMaxAcceleration(double maxAcceleration, int timeoutMs) {
        setMotionProfileMaxAcceleration(maxAcceleration);
    }

    @Override
    public void configMotionCurve(MotionCurveType motionCurveType, int curveStrength) {

    }

    @Override
    public void clearMotionProfileTrajectoryBuffer() {

    }

    @Override
    public String get_LastError() {
        return "Ghost motor, no errors!";
    }

    @Override
    public String get_Faults() {
        return "Ghost motor, no faults!";
    }

    @Override
    public String get_StickyFaults() {
        return "Ghost motor, no sticky faults!";
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
    public int getDeviceID() {
        return -1;
    }

    @Override
    public GreenControlMode get_ControlMode() {
        return controlMode;
    }

    @Override
    public void follow(IGreenMotor leader) {

    }

    @Override
    public double getSupplyCurrent() {
        return 0;
    }

    @Override
    public void restore_FactoryDefaults(int timeoutMs) {

    }

    @Override
    public boolean isVoltageCompensationEnabled() {
        return false;
    }

    @Override
    public int getQuadraturePosition() {
        return (int) desiredDemand[0];
    }

    @Override
    public void setQuadraturePosition(int quadraturePosition) {
        desiredDemand[2] = quadraturePosition;
    }

    @Override
    public int getPulseWidthPosition() {
        return (int) (absInitOffset + actualOutput[2]) % absMotorPPR;
    }

    @Override
    public boolean isFollower() {
        return false;
    }

    @Override
    public SoftLimitStatus getSoftLimitStatus() {
        return softLimitStatus;
    }

    @Override
    public void configControlFramePeriod(ControlFrame controlFrame, int periodms) {

    }
}
