package com.team1816.lib.util;

import edu.wpi.first.wpilibj.DataLogManager;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Contains basic functions that are used often in other utilities such as the EnhancedMotorChecker, Drive Conversions, and CheesyDriveHelper.
 */
public class Util {

    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    public static double deadBand(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static void cleanLogFiles() { // needs to be called after log
        var logPath = DataLogManager.getLogDir();
        long day = 1000 * 60 * 60 * 24;
        long now = System.currentTimeMillis();
        try (Stream<Path> stream = Files.list(Paths.get(logPath))) {
            var files = stream
                .filter(file -> !Files.isDirectory(file)) //No folders
                .filter(file -> file.toString().endsWith(".wpilog") ) //Only .wpiLog
                .filter(file -> file.toString().chars().filter(ch -> ch == '_').count() == 2 )
                .filter(file -> {
                    try {
                        return now - Files.getLastModifiedTime(file).toMillis() > day;
                    } catch (IOException e) {
                        return false;
                    }
                })
                .collect(Collectors.toSet());
            for (var file : files) {
                System.out.println("Deleting: " + file);
                Files.delete(file);
            }
        } catch (IOException e) {
            System.out.print(e);
        }
    }


    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(
        final List<Double> list,
        double value,
        double epsilon
    ) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double boundAngle0to360Degrees(double angle) {
        // Naive algorithm
        while (angle >= 360.0) {
            angle -= 360.0;
        }
        while (angle < 0.0) {
            angle += 360.0;
        }
        return angle;
    }
}
