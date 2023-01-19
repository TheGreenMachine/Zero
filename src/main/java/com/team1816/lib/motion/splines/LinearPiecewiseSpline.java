package com.team1816.lib.motion.splines;

import java.util.ArrayList;

/**
 * This class is also not in most senses a "spline" but represents a basic linearly continuous mapping of values
 *
 * @see Spline for documentation
 */

public class LinearPiecewiseSpline extends Spline {

    public static ArrayList<Double[]> coordinates;
    private static ArrayList<ArrayList<Double>> coefficients;

    public LinearPiecewiseSpline(ArrayList<Double[]> knotPoints) {
        super(knotPoints);
        coordinates = sort(knotPoints);
        coefficients = generateCoefficients();
    }

    @Override
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

    @Override
    public ArrayList<ArrayList<Double>> generateCoefficients() {
        ArrayList<ArrayList<Double>> lCoefficients = new ArrayList<>();
        for (int i = 1; i < coordinates.size(); i++) {
            ArrayList<Double> tempCoefficients = new ArrayList<>();
            double constant =
                coordinates.get(i - 1)[1] -
                    coordinates.get(i - 1)[0] *
                        (coordinates.get(i)[1] - coordinates.get(i - 1)[1]) /
                        (coordinates.get(i)[0] - coordinates.get(i - 1)[0]);
            double slope =
                (coordinates.get(i)[1] - coordinates.get(i - 1)[1]) /
                    (coordinates.get(i)[0] - coordinates.get(i - 1)[0]);
            tempCoefficients.add(constant);
            tempCoefficients.add(slope);
            lCoefficients.add(tempCoefficients);
        }
        return lCoefficients;
    }
}
