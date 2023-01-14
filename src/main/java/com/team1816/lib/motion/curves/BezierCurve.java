package com.team1816.lib.motion.curves;

import com.team1816.lib.motion.splines.NaturalCubicSpline;

import java.util.ArrayList;

/**
 * This class is an n-expandable framework for the implementation of a Bézier curve for motion characterization
 * and utilizes the Bernstein polynomial parametrization to define the curve and the points that consist of it.
 * This is different from a spline.
 *
 * @see com.team1816.lib.motion.splines.Spline
 */
public class BezierCurve {

    /**
     * Lightweight wrapper of point storage
     */
    public static class ControlPoint {

        /**
         * Properties
         */
        public double x;
        public double y;

        /**
         * Instantiates a default control point
         */
        public ControlPoint() {
            x = 0;
            y = 0;
        }

        /**
         * @param a x-coordinate
         * @param b y-coordinate of the control point
         */
        public ControlPoint(double a, double b) {
            x = a;
            y = b;
        }

        /**
         * Adds two control points together
         *
         * @param c (ControlPoint)
         */
        public void add(ControlPoint c) {
            x += c.x;
            y += c.y;
        }

        /**
         * Performs scalar multiplication by a value z
         *
         * @param z (double)
         */
        public void multiply(double z) {
            x *= z;
            y *= z;
        }

        /**
         * Gets the distance to another control point
         *
         * @param c (ControlPoint)
         * @return distance
         */
        public double getDistance(ControlPoint c) {
            return Math.hypot(c.x - x, c.y - y);
        }

        /**
         * Returns a Double[] array format of the control point
         *
         * @return {x, y}
         */
        public Double[] convertToDoubleArray() {
            return new Double[]{x, y};
        }

        /**
         * Standard toString() method
         *
         * @return {X: x, Y: y}
         */
        public String toString() {
            return "{X: " + x + ", Y: " + y + "}";
        }
    }

    /**
     * State
     */
    private ArrayList<ControlPoint> controlPoints; // sequentially defined list of control points that characterize the curve
    private ArrayList<Double> xCoefficients; // bernstein polynomial x co-efficients defined such that index refers to exponent
    private ArrayList<Double> yCoefficients; // bernstein polynomial y co-efficients defined such that index refers to exponent
    private NaturalCubicSpline LUT; // standard look-up table for distance arc-length calculations

    /**
     * Instantiates a default empty Bezier curve
     */
    public BezierCurve() {
        controlPoints = new ArrayList<>();
        xCoefficients = yCoefficients = new ArrayList<>();
        LUT = new NaturalCubicSpline(new ArrayList<Double[]>());
    }

    /**
     * Instantiates a Bézier curve based on a list of control points and calculates its look-up table
     *
     * @param arr
     */
    public BezierCurve(ArrayList<ControlPoint> arr) {
        controlPoints = arr;
        generateLookUpTable(100);
    }

    /**
     * Generates a look-up table based on a point resolution depending on the desired accuracy
     *
     * @param resolution computational resolution: number of points to consider for calculation
     */
    public void generateLookUpTable(int resolution) {
        ArrayList<Double[]> knotPoints = new ArrayList<>();
        for (int i = 0; i <= resolution; i++) {
            double t1 = (double) i / resolution;
            double dist = getPortionLength((i + 1) * resolution, 0, t1);
            knotPoints.add(new Double[]{dist, t1});
        }
        LUT = new NaturalCubicSpline(knotPoints);
    }

    /**
     * Standard linear interpolation of points
     *
     * @param p1 starting boundary Control Point
     * @param p2 ending boundary Control Point
     * @param t  parameter value to interpolate at
     * @return ControlPoint at t
     */
    private ControlPoint lerp(ControlPoint p1, ControlPoint p2, double t) {
        p1.multiply(1 - t);
        p2.multiply(t);
        p1.add(p2);
        return p1;
    }

    /**
     * Returns the point at the curve at parameter t
     *
     * @param t parameter
     * @return (ControlPoint)
     */
    public ControlPoint getValue(double t) {
        ControlPoint val = new ControlPoint();
        for (int i = 0; i <= controlPoints.size() - 1; i++) {
            ControlPoint c = controlPoints.get(i);
            c.multiply(
                Math.pow(t, i) *
                    Math.pow((1 - t), controlPoints.size() - i - 1) *
                    combination(controlPoints.size() - 1, i)
            );
            val.add(c);
        }
        return val;
    }

    /**
     * Gets a point on the curve a distance d away from the start
     *
     * @param d distance
     * @return (ControlPoint)
     */
    public ControlPoint getValueWithDistance(double d) {
        double t = Math.min(LUT.getValue(d), 1.0);
        return getValue(t);
    }

    /**
     * Multiplication utility for polynomial expansion
     *
     * @param n start
     * @param x exponentiated value
     * @return value
     */
    public double combination(int n, int x) {
        double ans = 1;
        for (int i = n; i > x; i--) {
            ans *= i;
        }
        for (int i = 1; i <= x; i++) {
            ans /= i;
        }
        return ans;
    }

    /**
     * Estimates length of the Bézier curve for a certain resolution
     *
     * @param resolution computational resolution
     * @return length
     */
    public double getLength(int resolution) {
        double distance = 0;
        ControlPoint p1 = controlPoints.get(0);
        ControlPoint p2 = new ControlPoint();
        for (int i = 0; i < resolution - 1; i++) {
            p2 = getValue((double) (i + 1) / resolution);
            distance += p1.getDistance(p2);
            p1 = p2;
        }
        return distance;
    }

    /**
     * Estimates the length of the Bézier curve over a certain portion
     *
     * @param resolution computational resolution
     * @param i          initial bounding parameter
     * @param f          final bounding parameter
     * @return length
     */
    public double getPortionLength(int resolution, double i, double f) {
        double distance = 0;
        ControlPoint p1 = getValue(i);
        ControlPoint p2 = new ControlPoint();
        for (double j = i; j <= f; j += 1.0 / resolution) {
            p2 = getValue(j);
            distance += p1.getDistance(p2);
            p1 = p2;
        }
        return distance;
    }
}
