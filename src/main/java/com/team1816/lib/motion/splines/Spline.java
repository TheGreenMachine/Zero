package com.team1816.lib.motion.splines;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

/**
 * This class is an optimized abstract container of the core splines package and is the framework of the polynomial
 * construction based on various knot points that control the spline.
 */
public abstract class Spline {

    /**
     * Properties
     */
    public static ArrayList<Double[]> coordinates;
    public static ArrayList<ArrayList<Double>> coefficients;

    /**
     * Initializes a spline based on its knotPoints
     *
     * @param knotPoints
     */
    protected Spline(ArrayList<Double[]> knotPoints) {
        coordinates = sort(knotPoints); // properly formats and checks coordinates
    }

    /**
     * Returns the value of the spline at a certain value
     *
     * @param input value to compute at
     * @return value
     */
    public double getValue(double input) {
        for (int i = 0; i < coordinates.size() - 1; i++) {
            if (input < coordinates.get(i + 1)[0]) { // this is because we want the value to hold until the next value
                double output = coordinates.get(i).length > 2 ? coordinates.get(i)[2] : 0; // use offsets if they exist
                for (int j = 0; j < coefficients.get(i).size(); j++) {
                    output += Math.pow(input, j) * coefficients.get(i).get(j);
                }
                return output;
            }
        }
        return coordinates.get(coordinates.size() - 1)[1];
    }

    /**
     * Generates coefficients for practically constant-time computation
     *
     * @return nested list of coefficients for each degree
     */
    public abstract ArrayList<ArrayList<Double>> generateCoefficients();

    /**
     * Optimally formats the knotPoints to avoid exceptions
     *
     * @param points knotPoints
     * @return sortedKnotPoints
     */
    protected ArrayList<Double[]> sort(ArrayList<Double[]> points) {
        ArrayList<Double[]> sorted = new ArrayList<>();
        HashMap<Double, Double[]> map = new HashMap<>();
        for (Double[] point : points) {
            if (point.length >= 2) { // ensures no malformed coordinates will be used
                map.put(point[0], point);
            }
        }
        List<Double> keys = new ArrayList<>(map.keySet());
        Collections.sort(keys);
        for (int i = 0; i < points.size(); i++) {
            sorted.add(map.get(keys.get(i)));
        }
        return sorted;
    }
}
