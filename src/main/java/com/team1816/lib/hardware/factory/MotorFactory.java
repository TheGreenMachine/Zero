package com.team1816.lib.hardware.factory;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.team1816.lib.hardware.MotorConfiguration;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.SubsystemConfig;
import com.team1816.lib.hardware.components.DeviceIdMismatchException;
import com.team1816.lib.hardware.components.motor.*;
import com.team1816.lib.hardware.components.motor.configurations.FeedbackDeviceType;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.RobotBase;

import javax.inject.Singleton;
import java.util.Map;

import static com.team1816.lib.subsystems.Subsystem.factory;

/**
 * A class to create and configure Falcon (TalonFX), TalonSRX, VictorSPX, SparkMax and GhostMotor objects.
 * Based on FRC Team 254 The Cheesy Poof's 2018 TalonSRXFactory
 */
@Singleton
public class MotorFactory {

    private static final int kTimeoutMs = RobotBase.isSimulation() ? 0 : 100;
    private static final int kTimeoutMsLONG = RobotBase.isSimulation() ? 0 : 200;

    // Factory default motor configs.

    public static NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
    public static double NEUTRAL_DEADBAND = 0.04;
    public static boolean ENABLE_CURRENT_LIMIT = false;
    public static boolean ENABLE_SOFT_LIMIT = false;
    public static boolean ENABLE_LIMIT_SWITCH = false;
    public static int FORWARD_SOFT_LIMIT = 0;
    public static int REVERSE_SOFT_LIMIT = 0;
    public static int CONTROL_FRAME_PERIOD_MS = 5;
    public static int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 1;
    public static double OPEN_LOOP_RAMP_RATE = 0.0;
    public static double CLOSED_LOOP_RAMP_RATE = 0.0;

    // Create a CANTalon with the default (out of the box) configuration.
    public static IGreenMotor createDefaultTalon(
        int id,
        String name,
        boolean isFalcon,
        SubsystemConfig subsystems,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId,
        String canBus
    ) {
        GreenLogger.log(
                "Creating " +
                        "TalonFX" +
                        " id:" +
                        id
        );

        return createTalon(
            id,
            name,
            isFalcon,
            subsystems,
            pidConfigList,
            remoteSensorId,
            canBus
        );
    }

    public static IGreenMotor createFollowerTalon(
        int id,
        String name,
        boolean isFalcon,
        IGreenMotor main,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        String canBus
    ) {
        final IGreenMotor talon = createTalon(
            id,
            name,
            isFalcon,
            subsystem,
            pidConfigList,
            -1, // never can have a remote sensor on Follower,
            canBus
        );
        GreenLogger.log(
            "Slaving talon on " + id + " to talon on " + main.getDeviceID()
        );
        talon.follow(main);
        return talon;
    }

    private static IGreenMotor createTalon(
        int id,
        String name,
        boolean isFalcon,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId,
        String canBus
    ) {
        IGreenMotor talon = isFalcon
            ? new LazyTalonFX(id, name, canBus)
            : new LazyTalonSRX(id, name);
        configMotor(talon, name, subsystem, pidConfigList, remoteSensorId);

        return talon;
    }

    public static IGreenMotor createGhostMotor(
        int maxVelTicks100ms,
        int absInitOffset,
        String name,
        SubsystemConfig subsystem
    ) {
        IGreenMotor motor = new GhostMotor(maxVelTicks100ms, absInitOffset, name);
        configMotor(motor, name, subsystem, null, -1);
        return motor;
    }

    public static IGreenMotor createDefaultVictor(
        int id,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId
    ) {
        return createVictor(id, name,subsystem,pidConfigList,remoteSensorId);
    }

    public static IGreenMotor createFollowerVictor(
        int id,
        String name,
        IGreenMotor main,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList
    ) {
        final IGreenMotor victor = createVictor(id, name,subsystem,pidConfigList,-1);
        GreenLogger.log(
            "Slaving victor on " + id + " to talon on " + main.getDeviceID()
        );
        victor.follow(main);
        return victor;
    }

    public static IGreenMotor createVictor(
        int id,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId
    ) {
        IGreenMotor victor = new LazyVictorSPX(id, name);

        configMotor(victor,name,subsystem,pidConfigList,remoteSensorId);
        victor.configReverseLimitSwitch(true);
        return victor;
    }

    public static IGreenMotor createSpark(
        int id,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        FeedbackDeviceType deviceType
    ) {

        IGreenMotor spark = new LazySparkMax(id, name);
        configMotor(spark,name,subsystem,pidConfigList,deviceType);
        return spark;
    }

    public static IGreenMotor createFollowerSpark(
        int id,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        FeedbackDeviceType deviceType,
        IGreenMotor leader
    ) {
        IGreenMotor followerSpark = createSpark(id,name,subsystem,pidConfigList,deviceType);
        followerSpark.follow(leader);
        followerSpark.setInverted(leader.getInverted());
        return followerSpark;
    }

    public static CANCoder createCanCoder(int canCoderID, String canBus, boolean invertCanCoder) {
        CANCoder canCoder = new CANCoder(canCoderID, canBus);
        if (factory.getConstant("resetFactoryDefaults", 0) > 0) {
            canCoder.configFactoryDefault(kTimeoutMs);
        }
        canCoder.configAllSettings(configureCanCoder(invertCanCoder), kTimeoutMsLONG);
        return canCoder;
    }

    private static void configMotor(
        IGreenMotor motor,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        int remoteSensorId
    ) {
        FeedbackDeviceType deviceType = FeedbackDeviceType.NO_SENSOR;
        if (remoteSensorId == 0) {
            deviceType = FeedbackDeviceType.REMOTE_SENSOR_0;
        } else if (remoteSensorId == 1) {
            deviceType = FeedbackDeviceType.REMOTE_SENSOR_1;
        }
        configMotor(motor, name, subsystem, pidConfigList, deviceType);
    }

    private static void configMotor(
        IGreenMotor motor,
        String name,
        SubsystemConfig subsystem,
        Map<String, PIDSlotConfiguration> pidConfigList,
        FeedbackDeviceType feedbackDeviceType
    ) {
        MotorConfiguration motorConfiguration = subsystem.motors.get(name);
        boolean isTalon = !(motor instanceof LazySparkMax || motor instanceof GhostMotor); // Talon also refers to VictorSPX, isCTRE just looks worse :)

        // Configuring feedback sensor
        motor.selectFeedbackSensor(feedbackDeviceType);

        // for newly attached motors only
        if (factory.getConstant("resetFactoryDefaults", 0) > 0) {
            GreenLogger.log("Resetting motor factory defaults");
            motor.restore_FactoryDefaults(kTimeoutMs);

            motor.configForwardSoftLimit(FORWARD_SOFT_LIMIT);
            motor.enableForwardSoftLimit(ENABLE_SOFT_LIMIT);

            motor.configReverseSoftLimit(REVERSE_SOFT_LIMIT);
            motor.enableReverseSoftLimit(ENABLE_SOFT_LIMIT);

            motor.config_NeutralDeadband(NEUTRAL_DEADBAND);

            motor.config_PeakOutputForward(1.0);
            motor.config_PeakOutputReverse(-1.0); // Use negative values for reverse peak output.

            motor.configOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.configClosedLoopRampRate(CLOSED_LOOP_RAMP_RATE);

            // CTRE exclusive configs
            if (isTalon) {
                ((BaseMotorController)motor).configVelocityMeasurementWindow(VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW);

                ((IMotorController)motor).configNominalOutputForward(0, kTimeoutMs); //TODO these should get remo
                ((IMotorController)motor).configNominalOutputReverse(0, kTimeoutMs);
            }
        }

        // PID configuration
        if (pidConfigList != null) {
            pidConfigList.forEach(
                (slot, slotConfig) -> {
                    int slotNum = ((int)slot.charAt(4)) - 48; //Minus 48 because charAt processes as a char, and digit ASCII values are themselves + 48
                    motor.set_kP(slotNum, slotConfig.kP != null ? slotConfig.kP : 0);
                    motor.set_kI(slotNum, slotConfig.kI != null ? slotConfig.kI : 0);
                    motor.set_kD(slotNum, slotConfig.kD != null ? slotConfig.kD : 0);
                    motor.set_kF(slotNum, slotConfig.kF != null ? slotConfig.kF : 0);
                    motor.set_iZone(slotNum, slotConfig.iZone != null ? slotConfig.iZone : 0);
                    motor.configAllowableErrorClosedLoop(slotNum, slotConfig.allowableError != null ? slotConfig.allowableError : 0);
                }
            );
        }

        // Setting to PID slot 0 and primary closed loop
        motor.selectPIDSlot(0,0);

        // Current limits
        motor.configCurrentLimit(
            new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
                motorConfiguration.currentLimit != null ? motorConfiguration.currentLimit : 40, //Default 40
                motorConfiguration.currentLimitThreshold != null ? motorConfiguration.currentLimitThreshold : 80, // Default 80
                motorConfiguration.currentLimitTriggerTime != null ? motorConfiguration.currentLimitTriggerTime : 1 // Default 1
            )
        );

        motor.enableLimitSwitches(ENABLE_LIMIT_SWITCH);

        // Setting up control frame with milliseconds (time is unused for sparks)
        motor.configControlFramePeriod(
            ControlFrame.Control_3_General,
            CONTROL_FRAME_PERIOD_MS
        );

        // inversion
        int id = motor.getDeviceID();
        if (id != motorConfiguration.id && RobotBase.isReal()) {
            GreenLogger.log(new DeviceIdMismatchException(name));
        } else {
            boolean invertMotor = motorConfiguration.invertMotor;
            if (invertMotor) {
                GreenLogger.log("        Inverting " + name + " with ID " + id);
            }
            motor.setInverted(invertMotor);

        }

        motor.setNeutralMode(NEUTRAL_MODE);

        // CTRE-Exclusive configurations
        if (isTalon) {
            //Casting might not work. Make sure to check.
            int remoteSensorId = feedbackDeviceType == FeedbackDeviceType.REMOTE_SENSOR_0 ? 0 : 1;
            ((BaseMotorController)motor).configRemoteFeedbackFilter(remoteSensorId, RemoteSensorSource.CANCoder, 0);
            ((BaseMotorController)motor).configClearPositionOnLimitF(false, kTimeoutMs);
            ((BaseMotorController)motor).configClearPositionOnLimitR(false, kTimeoutMs);

            // sensor phase inversion
            boolean invertSensorPhase = subsystem.invertSensorPhase.contains(name);
            if (invertSensorPhase) {
                GreenLogger.log(
                        "       Inverting sensor phase of " + name + " with ID " + id
                );
            }
            motor.setSensorPhase(invertSensorPhase);

        }

    }

    private static CANCoderConfiguration configureCanCoder(boolean invertCanCoder) {
        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.sensorDirection = invertCanCoder;
        canCoderConfig.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        return canCoderConfig;
    }

    private static SlotConfiguration toSlotConfiguration(
        PIDSlotConfiguration pidConfiguration
    ) {
        SlotConfiguration slotConfig = new SlotConfiguration();
        if (pidConfiguration != null) {
            if (pidConfiguration.kP != null) slotConfig.kP = pidConfiguration.kP;
            if (pidConfiguration.kI != null) slotConfig.kI = pidConfiguration.kI;
            if (pidConfiguration.kD != null) slotConfig.kD = pidConfiguration.kD;
            if (pidConfiguration.kP != null) slotConfig.kF = pidConfiguration.kF; // TODO should be kF notnull?
            if (pidConfiguration.iZone != null) slotConfig.integralZone =
                pidConfiguration.iZone;
            if (
                pidConfiguration.allowableError != null
            ) slotConfig.allowableClosedloopError = pidConfiguration.allowableError;
        }
        return slotConfig;
    }

    public enum MotorType {
        TalonSRX,
        Falcon,
        SparkMax,
        Ghost,
    }
}
