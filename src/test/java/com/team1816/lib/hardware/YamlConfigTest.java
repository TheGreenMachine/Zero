//package com.team1816.lib.hardware;
//
//import static org.junit.Assert.*;
//
//import com.team1816.frc2020.Robot;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.io.InputStream;
//import java.util.List;
//import org.junit.Ignore;
//import org.junit.Test;
//
//public class YamlConfigTest {
//
//    public static final double EPSILON = 1e-9;
//
//    private InputStream getResourceFile(String configName) {
//        return getClass()
//            .getClassLoader()
//            .getResourceAsStream(configName + ".config.yml");
//    }
//
//    private YamlConfig loadConfig(String configName) {
//        return YamlConfig.loadRaw(getResourceFile(configName));
//    }
//
//    @Test
//    public void subsystemConfig_merge() {
//        var base = loadConfig("test_base").subsystems.get("turret");
//        var active = loadConfig("test_active").subsystems.get("turret");
//        SubsystemConfig result = SubsystemConfig.merge(active, base);
//        System.out.println(result);
//
//        assertEquals("Base constant kP == 2.83", 2.83, result.constants.get("kP"), 0);
//        assertEquals(
//            "Overridden constant minPos == -374",
//            -374,
//            result.constants.get("minPos").intValue()
//        );
//        assertEquals(
//            "New constant newConstant == 34",
//            34,
//            result.constants.get("newConstant"),
//            0
//        );
//        assertEquals(
//            "Turret talon should be overridden to 13",
//            13,
//            result.talons.get("turret").intValue()
//        );
//        assertTrue("implemented == true (favors true)", result.isImplemented());
//    }
//
//    @Test(expected = ConfigIsAbstractException.class)
//    public void loadFromBase_throwsIfAbstract() throws ConfigIsAbstractException {
//        YamlConfig.loadFrom(getResourceFile("test_base"));
//    }
//
//    @Test
//    public void loadFromActive_doesNotThrow() throws ConfigIsAbstractException {
//        YamlConfig.loadFrom(getResourceFile("test_active"));
//    }
//
//    @Test
//    public void yamlConfig_merge() {
//        YamlConfig base = loadConfig("test_base");
//        YamlConfig active = loadConfig("test_active");
//        YamlConfig result = YamlConfig.merge(active, base);
//
//        verifyMergedConfig(result);
//    }
//
//    @Test
//    public void yamlConfig_autoMerge_ifExtends() throws ConfigIsAbstractException {
//        var configFile = getResourceFile("test_active");
//        YamlConfig config = YamlConfig.loadFrom(configFile);
//        verifyMergedConfig(config);
//    }
//
//    @Test
//    public void layer2_autoMerge() throws ConfigIsAbstractException {
//        var configFile = getResourceFile("test_layer2");
//        var config = YamlConfig.loadFrom(configFile);
//
//        verifyMergedConfig(config);
//
//        assertNotNull("collector != null", config.subsystems.get("collector"));
//        assertFalse(
//            "Turret not implemented",
//            config.subsystems.get("turret").isImplemented()
//        );
//        assertEquals(
//            "activeConstantOverridden == 18.16",
//            18.16,
//            config.getConstant("activeConstantOverridden"),
//            EPSILON
//        );
//        assertEquals(
//            "layer2Constant == 3.0",
//            3,
//            config.getConstant("layer2Constant"),
//            EPSILON
//        );
//    }
//
//    @Test(expected = ConfigIsAbstractException.class)
//    @Ignore // test_layer2 is not abstract
//    public void layer2_autoMerge_throwsIfAbstract() throws ConfigIsAbstractException {
//        var configFile = getResourceFile("test_layer2");
//        var config = YamlConfig.loadFrom(configFile);
//        System.out.println(config);
//    }
//
//    @Test
//    public void testImplementedOverride() {
//        mergeImplemented(true, false, true);
//        mergeImplemented(false, true, false);
//        mergeImplemented(true, true, true);
//        mergeImplemented(false, false, false);
//        mergeImplemented(null, null, false);
//        mergeImplemented(null, true, true);
//        mergeImplemented(true, null, true);
//    }
//
//    @Test
//    @Ignore // Only used to generate checking file
//    public void outputMergedYaml() throws ConfigIsAbstractException, IOException {
//        var configName = "alpha";
//        InputStream configFile =
//            Robot.class.getClassLoader().getResourceAsStream(configName + ".config.yml");
//        try (var writer = new FileWriter(configName + "_check.config.yml")) {
//            writer.write(YamlConfig.loadFrom(configFile).toString());
//        }
//    }
//
//    private void mergeImplemented(Boolean active, Boolean base, boolean result) {
//        var configActive = new SubsystemConfig(active);
//        var configBase = new SubsystemConfig(base);
//        var configResult = SubsystemConfig.merge(configActive, configBase);
//        assertEquals(result, configResult.isImplemented());
//    }
//
//    void verifyMergedConfig(YamlConfig config) {
//        System.out.println(config);
//
//        assertNotNull("Merged YAML config is not null", config);
//        assertNotNull(
//            "Subsystem config drivetrain is present",
//            config.subsystems.get("drivetrain")
//        );
//        assertNotNull(
//            "Subsystem config shooter is present",
//            config.subsystems.get("shooter")
//        );
//
//        assertEquals(
//            "Turret Talon ID == 13",
//            13,
//            config.subsystems.get("turret").talons.get("turret").intValue()
//        );
//        assertEquals(
//            "Overridden constant turret.minPos == -374",
//            -374,
//            config.subsystems.get("turret").constants.get("minPos").intValue()
//        );
//
//        assertEquals(
//            "Constant defined in base configuration baseConstant == 1",
//            1,
//            config.constants.get("baseConstant"),
//            0
//        );
//        assertEquals(
//            "Constant overridden in active config overriddenConstant == 0",
//            0,
//            config.constants.get("overriddenConstant"),
//            0
//        );
//        assertEquals(
//            "Constant defined in active configuration activeConstant == 399.42",
//            399.42,
//            config.constants.get("activeConstant"),
//            0
//        );
//
//        assertEquals("PCM ID is 8", 8, config.pcm.intValue());
//        assertTrue(
//            "invertMotor for invertMotorTest subsystem contains motorA and motorB",
//            config.subsystems
//                .get("invertMotorTest")
//                .invertMotor.containsAll(List.of("motorA", "motorB"))
//        );
//    }
//}
