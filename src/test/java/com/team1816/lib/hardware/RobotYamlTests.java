//package com.team1816.lib.hardware;
//
//import static org.junit.Assert.assertNotNull;
//
//import org.junit.Test;
//
//public class RobotYamlTests {
//
//    @Test
//    public void defaultYamlTest() {
//        loadConfig("default");
//    }
//
//    @Test
//    public void cheezeCurdYamlTest() {
//        loadConfig("CheezeCurd");
//    }
//
//    @Test
//    public void zodiacYamlTest() {
//        loadConfig("zodiac_swerve");
//    }
//
//    @Test
//    public void zodiacProYamlTest() {
//        loadConfig("zodiac_pro");
//    }
//
//    private void loadConfig(String configName) {
//        YamlConfig config = null;
//        try {
//            config =
//                YamlConfig.loadFrom(
//                    this.getClass()
//                        .getClassLoader()
//                        .getResourceAsStream(configName + ".config.yml")
//                );
//        } catch (ConfigIsAbstractException e) {
//            e.printStackTrace();
//        }
//        assertNotNull(config);
//        System.out.println(config);
//    }
//}
