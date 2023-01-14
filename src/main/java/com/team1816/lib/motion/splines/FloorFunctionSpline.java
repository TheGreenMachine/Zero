package com.team1816.lib.motion.splines;

import java.util.ArrayList;

/**
 * This class is not a spline in a traditional sense of a mathematical function, but represents a floor function mapping of values.
 *
 * @see Spline for documentation
 */

public class FloorFunctionSpline extends Spline {

    public static ArrayList<Double[]> coordinates;
    public static ArrayList<ArrayList<Double>> coefficients;

    public FloorFunctionSpline(ArrayList<Double[]> knotPoints) {
        super(knotPoints);
        coordinates = sort(knotPoints);
        coefficients = generateCoefficients();
    }

    @Override
    public double getValue(double input) {
        for (int i = 0; i < coordinates.size() - 1; i++) {
            if (input < coordinates.get(i + 1)[0]) { // this is because we want the value to hold until the next value
                double output = coordinates.get(i).length > 2 ? coordinates.get(i)[2] : 0; // use offsets if they exist
                if (coefficients.size() <= 0) {
                    coefficients = generateCoefficients();
                }
                for (int j = 0; j < coefficients.get(i).size(); j++) {
                    output += Math.pow(input, j) * coefficients.get(i).get(j);
                }
                return output;
            }
        }
        return coordinates.get(coordinates.size() - 1)[1];
    }

    @Override
    public ArrayList<ArrayList<Double>> generateCoefficients() {
        ArrayList<ArrayList<Double>> fCoefficients = new ArrayList<>();
        for (int i = 0; i < coordinates.size(); i++) {
            ArrayList<Double> tempCoefficients = new ArrayList<>();
            tempCoefficients.add(coordinates.get(i)[1]);
            fCoefficients.add(tempCoefficients);
        }
        return fCoefficients;
    }
}
