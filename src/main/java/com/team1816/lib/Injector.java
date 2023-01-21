package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Module;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper for the GUICE injector to register modules and straighten instantiation pathways for run-time optimization
 */
public class Injector {

    private static com.google.inject.Injector _injector;

    private static List<Module> _modules = new ArrayList<>();

    /**
     * This method will initial the injector using the items in the
     * lib module and the passed in season module
     *
     * @param module This is the season module to register
     */
    public static void registerModule(Module module) {
        _modules.add(module);
    }

    /**
     * Registers an instance as a module
     *
     * @param instance
     * @param <T>
     */
    public static <T> void register(T instance) {
        _modules.add(
            new AbstractModule() {
                @Override
                protected void configure() {
                    bind((Class) instance.getClass()).toInstance(instance);
                }
            }
        );
    }

    /**
     * Registers a class as a module
     *
     * @param type
     * @param instance
     * @param <T>
     */
    public static <T> void register(Class<T> type, Class<? extends T> instance) {
        _modules.add(
            new AbstractModule() {
                @Override
                protected void configure() {
                    bind(type).to(instance);
                }
            }
        );
    }

    /**
     * Returns a module based on its associated class
     *
     * @param type
     * @param <T>
     * @return
     */
    public static <T> T get(Class<T> type) {
        // on first retrieval lock in the modules and create injector
        if (_injector == null) {
            _modules.add(new LibModule());
            _injector = Guice.createInjector(_modules);
        }
        return _injector.getInstance(type);
    }
}
