package com.team1816.lib.util;

import java.util.List;

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

    /**
     * Returns whichever int in an array a given int is closest to
     *
     * @see <a href="https://stackoverflow.com/a/13318769">Credit To Chris Hayes under the WTFPL license</a>
     * @param numbers The array of ints
     * @param value The given int
     * @return The element that the given value is closest to
     */
    public static int closestTo(Integer[] numbers, int value) {
        int distance = Math.abs(numbers[0] - value);
        int idx = 0;
        for(int c = 1; c < numbers.length; c++){
            int cdistance = Math.abs(numbers[c] - value);
            if(cdistance < distance){
                idx = c;
                distance = cdistance;
            }
        }
        return numbers[idx];
    }
}
