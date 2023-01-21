package com.team1816.lib.controlboard;

import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.representer.Representer;

import java.io.InputStream;

/**
 * The yaml parsing integration for a control board.
 *
 * @see ControlBoardBridge
 */
public class ControlBoardYamlConfig {

    private static final Yaml FORMATTER = new Yaml();

    static {
        FORMATTER.setBeanAccess(BeanAccess.FIELD);
    }

    public static ControlBoardConfig loadFrom(InputStream input) {
        return loadInternal(input);
    }

    static ControlBoardConfig loadInternal(InputStream input) {
        return loadRaw(input);
    }

    static ControlBoardConfig loadRaw(InputStream input) {
        Representer representer = new Representer();
        representer.getPropertyUtils().setSkipMissingProperties(true);
        Yaml yaml = new Yaml(new Constructor(ControlBoardConfig.class), representer);
        yaml.setBeanAccess(BeanAccess.FIELD);

        return yaml.load(input);
    }

    @Override
    public String toString() {
        return FORMATTER.dump(this);
    }
}
