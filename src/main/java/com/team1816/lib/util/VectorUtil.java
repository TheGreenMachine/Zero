package com.team1816.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A simple vector utility for basic products
 */
public class VectorUtil {

    /**
     * Returns the angle between two translations
     *
     * @param a Translation2d
     * @param b Translation2d
     * @return angle (radians)
     */
    public static double getAngleBetween(Translation2d a, Translation2d b) {
        double dot = (a.getNorm() * b.getNorm() == 0)
            ? 0
            : Math.acos(
            (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm())
        );
        double cross = crossProduct(a, b);
        if (cross > 0) {
            dot *= -1;
        }
        return dot;
    }

    /**
     * Simple cross product of two translations
     *
     * @param a Translation2d
     * @param b Translation2d
     * @return product
     */
    private static double crossProduct(Translation2d a, Translation2d b) {
        double[] vect_A = {a.getX(), a.getY(), 0};
        double[] vect_B = {b.getX(), b.getY(), 0};
        return vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
    }
}
