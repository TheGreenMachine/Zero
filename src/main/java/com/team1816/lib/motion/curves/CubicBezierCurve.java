package com.team1816.lib.motion.curves;

import com.team1816.lib.motion.splines.NaturalCubicSpline;

import java.util.ArrayList;
import java.util.Collections;

/**
 * This class is a lighter fixed-case computational environment for a cubic Bézier curve. For documentation
 *
 * @see BezierCurve
 */
public class CubicBezierCurve {

    public static class ControlPoint {

        public double x;
        public double y;

        public ControlPoint() {
            x = 0;
            y = 0;
        }

        public ControlPoint(double a, double b) {
            x = a;
            y = b;
        }

        public void add(ControlPoint c) {
            x += c.x;
            y += c.x;
        }

        public void multiply(double z) {
            x *= z;
            y *= z;
        }

        public double getDistance(ControlPoint c) {
            return Math.hypot(c.x - x, c.y - y);
        }

        public Double[] convertToDoubleArray() {
            return new Double[]{x, y};
        }
    }

    private ArrayList<ControlPoint> controlPoints; // defined sequentially
    private ArrayList<Double> xCoefficients; // defined such that index refers to exponent
    private ArrayList<Double> yCoefficients; // defined such that index refers to exponent
    private NaturalCubicSpline LUT;

    public CubicBezierCurve() {
        controlPoints = new ArrayList<>();
        xCoefficients = yCoefficients = new ArrayList<>();
    }

    public CubicBezierCurve(
        ControlPoint p0,
        ControlPoint p1,
        ControlPoint p2,
        ControlPoint p3
    ) {
        controlPoints = new ArrayList<>();
        controlPoints.add(p0);
        controlPoints.add(p1);
        controlPoints.add(p2);
        controlPoints.add(p3);
        xCoefficients.add(-1 * p0.x + 3 * p1.x - 3 * p2.x + p3.x);
        yCoefficients.add(-1 * p0.y + 3 * p1.y - 3 * p2.y + p3.y);
        xCoefficients.add(3 * p0.x - 6 * p1.x + 3 * p2.x);
        yCoefficients.add(3 * p0.y - 6 * p1.y + 3 * p2.y);
        xCoefficients.add((-3) * p0.x + 3 * p1.x);
        yCoefficients.add((-3) * p0.y + 3 * p1.y);
        xCoefficients.add(p0.x);
        yCoefficients.add(p0.y);
        Collections.reverse(xCoefficients);
        Collections.reverse(yCoefficients);
        generateLookUpTable(100);
    }

    public void generateLookUpTable(int resolution) {
        ArrayList<Double[]> knotPoints = new ArrayList<>();
        for (int i = 0; i <= resolution; i++) {
            double t1 = (double) i / resolution;
            double dist = getPortionLength((i + 1) * resolution, 0, t1);
            knotPoints.add(new Double[]{dist, t1});
        }
        LUT = new NaturalCubicSpline(knotPoints);
    }

    private ControlPoint lerp(ControlPoint p1, ControlPoint p2, double t) {
        p1.multiply(1 - t);
        p2.multiply(t);
        p1.add(p2);
        return p1;
    }

    public ControlPoint getValue(double t) {
        double x = 0, y = 0;
        for (int i = 0; i < xCoefficients.size() && i < yCoefficients.size(); i++) {
            x += xCoefficients.get(i) * Math.pow(t, i);
            y += yCoefficients.get(i) * Math.pow(t, i);
        }
        return new ControlPoint(x, y);
    }

    public ControlPoint getValueWithDistance(double d) {
        double t = Math.min(LUT.getValue(d), 1.0);
        return getValue(t);
    }

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
