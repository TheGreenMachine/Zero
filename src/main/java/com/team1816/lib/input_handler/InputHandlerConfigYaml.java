package com.team1816.lib.input_handler;

import com.team1816.lib.input_handler.InputHandlerBridge;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.representer.Representer;

import java.io.InputStream;

/**
 * The yaml parsing integration for the input handler
 *
 * @see InputHandlerBridge
 */
public class InputHandlerConfigYaml {
    private static final Yaml FORMATTER = new Yaml();

    static {
        FORMATTER.setBeanAccess(BeanAccess.FIELD);
    }

    public static InputHandlerConfig loadFrom(InputStream input) {
        return loadInternal(input);
    }

    static InputHandlerConfig loadInternal(InputStream input) {
        return loadRaw(input);
    }

    static InputHandlerConfig loadRaw(InputStream input) {
        Representer representer = new Representer();
        representer.getPropertyUtils().setSkipMissingProperties(true);
        Yaml yaml = new Yaml(new Constructor(InputHandlerConfig.class), representer);
        yaml.setBeanAccess(BeanAccess.FIELD);

        return yaml.load(input);
    }

    @Override
    public String toString() {
        return FORMATTER.dump(this);
    }
}
