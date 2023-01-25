package com.team1816.lib.motion.splines;

import java.util.ArrayList;

/**
 * This class models a quintic spline offering four degrees of differentiable continuity for a smooth mapping via
 * direct coefficient computation.
 *
 * @see Spline for documentation
 */

public class QuinticSpline extends Spline {

    public static ArrayList<Double[]> coordinates;
    private static ArrayList<ArrayList<Double>> coefficients;

    public QuinticSpline(ArrayList<Double[]> knotPoints) {
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

    // This method is mathematically accurate, yet due to the need for floating point accuracy to at least 20 places, it is not advisable for practical application
    @Override
    public ArrayList<ArrayList<Double>> generateCoefficients() { // a less mathematically intense process, relies on the cubic spline for derivatives
        NaturalCubicSpline spline = new NaturalCubicSpline(coordinates);
        ArrayList<ArrayList<Double>> qCoefficients = new ArrayList<>();
        for (int i = 1; i < coordinates.size(); i++) { // compression to limit floating point calculation
            ArrayList<Double> tempCoefficients = new ArrayList<>();

            double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
            double xi, xf, yi, yf, vi, vf, ai, af;

            xi = coordinates.get(i - 1)[0];
            xf = coordinates.get(i)[0];
            yi = coordinates.get(i - 1)[1];
            yf = coordinates.get(i)[1];
            vi = spline.getDerivativeAtKnotPointIndex(i - 1);
            vf = spline.getDerivativeAtKnotPointIndex(i);
            ai = spline.getSecondDerivativeAtKnotPointIndex(i - 1);
            af = spline.getSecondDerivativeAtKnotPointIndex(i);

            a =
                (double) 1 /
                    (2 * Math.pow((xf - xi), 5)) *
                    (
                        Math.pow((xf - xi), 2) *
                            (af - ai) -
                            6 *
                                (xf - xi) *
                                (vf + vi) +
                            12 *
                                (yf - yi)
                    );
            b =
                (double) 1 /
                    (2 * Math.pow((xf - xi), 5)) *
                    (
                        -1 *
                            Math.pow((xf - xi), 2) *
                            (af * (2 * xf + 3 * xi) - ai * (3 * xf + 2 * xi)) +
                            2 *
                                (xf - xi) *
                                (vf * (7 * xf + 8 * xi) + vi * (8 * xf + 7 * xi)) -
                            30 *
                                (yf - yi) *
                                (xf + xi)
                    );
            c =
                (double) 1 /
                    (2 * Math.pow((xf - xi), 5)) *
                    (
                        Math.pow((xf - xi), 2) *
                            (
                                af *
                                    (3 * Math.pow(xi, 2) + 6 * xi * xf * Math.pow(xf, 2)) -
                                    ai *
                                        (Math.pow(xi, 2) + 6 * xi * xf + 3 * Math.pow(xf, 2))
                            ) -
                            4 *
                                (xf - xi) *
                                (
                                    vi *
                                        (2 * Math.pow(xi, 2) + 10 * xi * xf + 3 * Math.pow(xf, 2)) +
                                        vf *
                                            (3 * Math.pow(xi, 2) + 10 * xi * xf + 2 * Math.pow(xf, 2))
                                ) +
                            20 *
                                (yf - yi) *
                                (Math.pow(xf, 2) + 4 * xf * xi + Math.pow(xi, 2))
                    );
            d =
                (double) ai /
                    2 -
                    10 *
                        a *
                        Math.pow(xi, 3) -
                    6 *
                        a *
                        Math.pow(xi, 2) -
                    3 *
                        c *
                        xi;
            e =
                (double) vi -
                    5 *
                        a *
                        Math.pow(xi, 4) -
                    4 *
                        b *
                        Math.pow(xi, 3) -
                    3 *
                        c *
                        Math.pow(xi, 2) -
                    2 *
                        d *
                        xi;
            f =
                (double) yi -
                    a *
                        Math.pow(xi, 5) -
                    b *
                        Math.pow(xi, 4) -
                    c *
                        Math.pow(xi, 3) -
                    d *
                        Math.pow(xi, 2) -
                    e *
                        xi;

            tempCoefficients.add(f);
            tempCoefficients.add(e);
            tempCoefficients.add(d);
            tempCoefficients.add(c);
            tempCoefficients.add(b);
            tempCoefficients.add(a);

            qCoefficients.add(tempCoefficients);
        }
        return qCoefficients;
    }
}
