package com.team1816.lib.hardware.factory;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.*;
import com.team1816.lib.hardware.components.gyro.GhostPigeonIMU;
import com.team1816.lib.hardware.components.gyro.IPigeonIMU;
import com.team1816.lib.hardware.components.gyro.Pigeon2Impl;
import com.team1816.lib.hardware.components.gyro.PigeonIMUImpl;
import com.team1816.lib.hardware.components.ledManager.CANdleImpl;
import com.team1816.lib.hardware.components.ledManager.CanifierImpl;
import com.team1816.lib.hardware.components.ledManager.GhostLEDManager;
import com.team1816.lib.hardware.components.ledManager.ILEDManager;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.LazySparkMax;
import com.team1816.lib.hardware.components.pcm.*;
import com.team1816.lib.hardware.components.sensor.GhostProximitySensor;
import com.team1816.lib.hardware.components.sensor.IProximitySensor;
import com.team1816.lib.hardware.components.sensor.ProximitySensor;
import com.team1816.lib.subsystems.drive.SwerveModule;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;

import javax.annotation.Nonnull;
import java.util.Map;
import java.util.Objects;

/**
 * This class employs the MotorFactory and SensorFactory with yaml integrations and is the initial entry point to
 * create and initialize any and all components on a robot.
 *
 * @see MotorFactory
 * @see SensorFactory
 * @see YamlConfig
 */
@Singleton
public class RobotFactory {

    private RobotConfiguration config;

    public RobotFactory() {
        var robotName = System.getenv("ROBOT_NAME");
        if (robotName == null) {
            robotName = "default";
            DriverStation.reportWarning(
                "ROBOT_NAME environment variable not defined, falling back to default.config.yml!",
                false
            );
        }
        System.out.println("Loading Config for " + robotName);
        try {
            config =
                YamlConfig.loadFrom(
                    this.getClass()
                        .getClassLoader()
                        .getResourceAsStream("yaml/" + robotName + ".config.yml")
                );
        } catch (Exception e) {
            DriverStation.reportError("Yaml Config error!", e.getStackTrace());
        }
    }

    public IGreenMotor getMotor(
        String subsystemName,
        String name,
        Map<String, PIDSlotConfiguration> pidConfigs,
        int remoteSensorId
    ) {
        IGreenMotor motor = null;
        var subsystem = getSubsystem(subsystemName);

        // Identifying motor type
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.talons, name)) {
                motor =
                    MotorFactory.createDefaultTalon(
                        subsystem.talons.get(name),
                        name,
                        false,
                        subsystem,
                        pidConfigs,
                        remoteSensorId,
                        config.infrastructure.canBusName
                    );
            } else if (isHardwareValid(subsystem.falcons, name)) {
                motor =
                    MotorFactory.createDefaultTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        subsystem,
                        pidConfigs,
                        remoteSensorId,
                        config.infrastructure.canBusName
                    );
            } else if (isHardwareValid(subsystem.sparkmaxes, name)) {
                motor =
                    MotorFactory.createSpark(
                        subsystem.sparkmaxes.get(name),
                        name,
                        subsystem,
                        pidConfigs
                    );
            }
            // Never make the victor a main
        }

        // report creation of motor
        if (motor == null) {
            reportGhostWarning("Motor", subsystemName, name);
            motor =
                MotorFactory.createGhostMotor(
                    (int) (getConstant(subsystemName, "maxVelTicks100ms", 1, false)),
                    0,
                    name,
                    subsystem
                );
        } else {
            GreenLogger.log(
                "Created " +
                    motor.getClass().getSimpleName() +
                    " id:" +
                    motor.getDeviceID()
            );
        }

        return motor;
    }

    public IGreenMotor getMotor(String subsystemName, String name) {
        return getMotor(subsystemName, name, getSubsystem(subsystemName).pidConfig, -1);
    }

    public IGreenMotor getFollowerMotor(
        String subsystemName,
        String name,
        IGreenMotor main
    ) {
        IGreenMotor followerMotor = null;
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented && main != null) {
            if (isHardwareValid(subsystem.talons, name)) {
                // Talons must be following another Talon, cannot follow a Victor.
                followerMotor =
                    MotorFactory.createFollowerTalon(
                        subsystem.talons.get(name),
                        name,
                        false,
                        main,
                        subsystem,
                        subsystem.pidConfig,
                        config.infrastructure.canBusName
                    );
            } else if (isHardwareValid(subsystem.falcons, name)) {
                followerMotor =
                    MotorFactory.createFollowerTalon(
                        subsystem.falcons.get(name),
                        name,
                        true,
                        main,
                        subsystem,
                        subsystem.pidConfig,
                        config.infrastructure.canBusName
                    );
            } else if (isHardwareValid(subsystem.sparkmaxes, name)) {
                followerMotor =
                    MotorFactory.createSpark(
                        subsystem.sparkmaxes.get(name),
                        name,
                        subsystem
                    );
                ((LazySparkMax) followerMotor).follow(
                    main,
                    subsystem.invertMotor.contains(name)
                );
                followerMotor.setInverted(main.getInverted());
            } else if (isHardwareValid(subsystem.victors, name)) {
                // Victors can follow Talons or another Victor.
                followerMotor =
                    MotorFactory.createFollowerVictor(
                        subsystem.victors.get(name),
                        name,
                        main
                    );
            }
        }
        if (followerMotor == null) {
            if (subsystem.implemented) reportGhostWarning("Motor", subsystemName, name);
            followerMotor =
                MotorFactory.createGhostMotor(
                    (int) getConstant(subsystemName, "maxVelTicks100ms"),
                    0,
                    name,
                    subsystem
                );
        }
        if (main != null) {
            followerMotor.setInverted(main.getInverted());
        }
        return followerMotor;
    }

    public SwerveModule getSwerveModule(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        ModuleConfiguration module = subsystem.swerveModules.modules.get(name);
        if (module == null) {
            DriverStation.reportError(
                "No swerve module with name " + name + " subsystem " + subsystemName,
                true
            );
            return null;
        }

        var moduleConfig = new SwerveModule.ModuleConfig();
        moduleConfig.moduleName = name;
        moduleConfig.azimuthMotorName = module.azimuth; // getAzimuth and drive give ID I think - not the module name (ex: leftRear)
        moduleConfig.azimuthPid =
            getPidSlotConfig(subsystemName, "slot0", PIDConfig.Azimuth);
        moduleConfig.driveMotorName = module.drive;
        moduleConfig.drivePid = getPidSlotConfig(subsystemName, "slot0", PIDConfig.Drive);
        moduleConfig.azimuthEncoderHomeOffset = module.constants.get("encoderOffset");

        var canCoder = getCanCoder(subsystemName, name);

        return new SwerveModule(subsystemName, moduleConfig, canCoder);
    }

    public CANCoder getCanCoder(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        var module = subsystem.swerveModules.modules.get(name);
        CANCoder canCoder = null;
        if (
            module != null &&
                module.canCoder != null &&
                subsystem.canCoders.get(module.canCoder) >= 0
        ) {
            canCoder =
                MotorFactory.createCanCoder(
                    subsystem.canCoders.get(module.canCoder),
                    config.infrastructure.canBusName,
                    subsystem.canCoders.get(subsystem.invertCanCoder) != null &&
                        subsystem.invertCanCoder.contains(module.canCoder)
                );
        }

        // purposefully return null so that swerve modules default to quad encoders
        return canCoder;
    }

    @Nonnull
    public ISolenoid getSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.solenoids, name) && isPcmEnabled()) {
                return new SolenoidImpl(
                    config.infrastructure.pcmId,
                    config.infrastructure.pcmIsRev
                        ? PneumaticsModuleType.REVPH
                        : PneumaticsModuleType.CTREPCM,
                    subsystem.solenoids.get(name)
                );
            }
            reportGhostWarning("Solenoid", subsystemName, name);
        }
        return new GhostSolenoid();
    }

    @Nonnull
    public IDoubleSolenoid getDoubleSolenoid(String subsystemName, String name) {
        var subsystem = getSubsystem(subsystemName);
        if (getSubsystem(subsystemName).doubleSolenoids != null) {
            DoubleSolenoidConfig solenoidConfig = getSubsystem(subsystemName)
                .doubleSolenoids.get(name);
            if (
                subsystem.implemented &&
                    solenoidConfig != null &&
                    isHardwareValid(solenoidConfig.forward) &&
                    isHardwareValid(solenoidConfig.reverse) &&
                    isPcmEnabled()
            ) {
                return new DoubleSolenoidImpl(
                    config.infrastructure.pcmId,
                    PneumaticsModuleType.REVPH,
                    solenoidConfig.forward,
                    solenoidConfig.reverse
                );
            }
        }
        reportGhostWarning("DoubleSolenoid", subsystemName, name);
        return new GhostDoubleSolenoid();
    }

    public ILEDManager getLEDManager(String subsystemName) {
        var subsystem = getSubsystem(subsystemName);
        ILEDManager ledManager = null;
        if (subsystem.implemented) {
            if (isHardwareValid(subsystem.canifier)) {
                ledManager = new CanifierImpl(subsystem.canifier);
            } else if (isHardwareValid(subsystem.candle)) {
                ledManager =
                    new CANdleImpl(
                        subsystem.candle,
                        config.infrastructure.canBusName
                    );
            }
            if (ledManager != null) {
                ledManager.configFactoryDefault();
                ledManager.configStatusLedState(true);
                ledManager.configLOSBehavior(false);
                ledManager.configLEDType(CANdle.LEDStripType.BRG); // type config of the strip we use rn
                ledManager.configBrightnessScalar(1);
                return ledManager;
            }
            reportGhostWarning("LEDManager", subsystemName, "");
        }

        return new GhostLEDManager();
    }

    public ICompressor getCompressor() {
        if (isPcmEnabled() && config.infrastructure.compressorEnabled) {
            PneumaticsModuleType pcmType = config.infrastructure.pcmIsRev
                ? PneumaticsModuleType.REVPH
                : PneumaticsModuleType.CTREPCM;
            return new CompressorImpl(getPcmId(), pcmType);
        }
        reportGhostWarning("Compressor", "ROOT", "on PCM ID " + getPcmId()); // root?
        return new GhostCompressor();
    }

    private boolean isHardwareValid(Map<String, Integer> map, String name) {
        if (map != null) {
            Integer hardwareId = map.get(name);
            return hardwareId != null && hardwareId > -1 && RobotBase.isReal();
        }
        return false;
    }

    private boolean isHardwareValid(Integer hardwareId) {
        return hardwareId != null && hardwareId > -1 && RobotBase.isReal();
    }

    public Double getConstant(String name) {
        return getConstant(name, 0);
    }

    public Map<String, Double> getConstants() {
        return config.constants;
    }

    public SubsystemConfig getSubsystem(String subsystemName) {
        if (config.subsystems.containsKey(subsystemName)) {
            var subsystem = config.subsystems.get(subsystemName);
            if (subsystem == null) {
                subsystem = new SubsystemConfig();
                subsystem.implemented = false;
                GreenLogger.log("Subsystem not defined: " + subsystemName);
            }
            return subsystem;
        }
        SubsystemConfig subsystem = new SubsystemConfig();
        subsystem.implemented = false;
        return subsystem;
    }

    public double getConstant(String name, double defaultVal) {
        if (getConstants() == null || !getConstants().containsKey(name)) {
            DriverStation.reportWarning("Yaml constants:" + name + " missing", true);
            return defaultVal;
        }
        return getConstants().get(name);
    }

    public String getControlBoard() {
        return Objects.requireNonNullElse(config.controlboard, "empty");
    }

    public double getConstant(String subsystemName, String name) {
        return getConstant(subsystemName, name, 0.0);
    }

    public double getConstant(String subsystemName, String name, double defaultVal) {
        return getConstant(subsystemName, name, defaultVal, true);
    }

    public double getConstant(
        String subsystemName,
        String name,
        double defaultVal,
        boolean showWarning
    ) {
        if (!getSubsystem(subsystemName).implemented) {
            return defaultVal;
        }
        if (
            getSubsystem(subsystemName).constants == null ||
                !getSubsystem(subsystemName).constants.containsKey(name)
        ) {
            if (showWarning) {
                DriverStation.reportWarning(
                    "Yaml: subsystem \"" +
                        subsystemName +
                        "\" constant \"" +
                        name +
                        "\" missing",
                    defaultVal == 0
                );
            }
            return defaultVal;
        }
        return getSubsystem(subsystemName).constants.get(name);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName) {
        return getPidSlotConfig(subsystemName, "slot0", PIDConfig.Generic);
    }

    public PIDSlotConfiguration getPidSlotConfig(String subsystemName, String slot) {
        return getPidSlotConfig(subsystemName, slot, PIDConfig.Generic);
    }

    public PIDSlotConfiguration getPidSlotConfig(
        String subsystemName,
        String slot,
        PIDConfig configType
    ) {
        var subsystem = getSubsystem(subsystemName);
        Map<String, PIDSlotConfiguration> config = null;
        if (subsystem.implemented) {
            switch (configType) {
                case Azimuth:
                    config = subsystem.swerveModules.azimuthPID;
                    break;
                case Drive:
                    config = subsystem.swerveModules.drivePID;
                    break;
                case Generic:
                    config = subsystem.pidConfig;
                    break;
            }
        }
        if (config != null && config.get(slot) != null) return config.get(slot);
        else {
            if (subsystem.implemented) {
                DriverStation.reportError(
                    "pidConfig missing for " + subsystemName + " " + slot,
                    true
                );
                return null;
            } else {
                // return a default config if not implemented
                PIDSlotConfiguration pidSlotConfiguration = new PIDSlotConfiguration();
                pidSlotConfiguration.kP = 0.0;
                pidSlotConfiguration.kI = 0.0;
                pidSlotConfiguration.kD = 0.0;
                pidSlotConfiguration.kF = 0.0;
                pidSlotConfiguration.iZone = 0;
                pidSlotConfiguration.allowableError = 0.0;
                return pidSlotConfiguration;
            }
        }
    }

    public PowerDistribution getPd() {
        return new PowerDistribution(
            config.infrastructure.pdId,
            config.infrastructure.pdIsRev
                ? PowerDistribution.ModuleType.kRev
                : PowerDistribution.ModuleType.kCTRE
        );
    }

    public IPigeonIMU getPigeon() {
        int id = config.infrastructure.pigeonId;
        IPigeonIMU pigeon;
        if (!isHardwareValid(id)) {
            pigeon = new GhostPigeonIMU(id);
        } else if (config.infrastructure.isPigeon2) {
            GreenLogger.log("Using Pigeon 2 for id: " + id);
            pigeon = new Pigeon2Impl(id, config.infrastructure.canBusName);
        } else {
            GreenLogger.log("Using old Pigeon for id: " + id);
            pigeon = new PigeonIMUImpl(id);
        }
        if (getConstant("resetFactoryDefaults") > 0) {
            pigeon.configFactoryDefault();
            pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 100);
            pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 100);
        }
        return pigeon;
    }

    public IProximitySensor getProximitySensor(String name) {
        if (config.infrastructure.proximitySensors == null) {
            return new GhostProximitySensor();
        }
        int id = config.infrastructure.proximitySensors.getOrDefault(name, -1);
        if (id < 0) {
            GreenLogger.log("Incorrect Name: Proximity sensor not found, using ghost!");
            return new GhostProximitySensor();
        }
        GreenLogger.log("Creating Proximity Sensor: " + name + " at port: " + id);
        return new ProximitySensor(name, config.infrastructure.proximitySensors.get(name));
    }

    public int getPcmId() {
        if (config.infrastructure == null && config.infrastructure.pcmId == null) return -1;
        return config.infrastructure.pcmId;
    }

    public boolean isPcmEnabled() {
        return getPcmId() > -1;
    }

    public boolean isCompressorEnabled() {
        return config.infrastructure.compressorEnabled;
    }

    private void reportGhostWarning(
        String type,
        String subsystemName,
        String componentName
    ) {
        GreenLogger.log(
            "  " +
                type +
                " \"" +
                componentName +
                "\" invalid in Yaml for subsystem \"" +
                subsystemName +
                "\", using ghost!"
        );
    }

    private enum PIDConfig {
        Azimuth,
        Drive,
        Generic,
    }
}
