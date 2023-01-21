package com.team1816.lib.util;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

/**
 * An adaptation of Apache commons' kalman filter for robot state estimation
 */

public class RSKalmanFilter {

    public double dt = Constants.kLooperDt;
    public double measurementNoise = 0.8;

    Array2DRowRealMatrix A = new Array2DRowRealMatrix(
        new double[][]{
            {1d, 0d, dt, 0d},
            {0d, 1d, 0d, dt},
            {0d, 0d, 1d, 0d},
            {0d, 0d, 0d, 1d},
        }
    );
    Array2DRowRealMatrix B = new Array2DRowRealMatrix(
        new double[][]{
            {Math.pow(dt, 2d) / 2d},
            {Math.pow(dt, 2d) / 2d},
            {dt},
            {dt},
        }
    );

    Array2DRowRealMatrix H = new Array2DRowRealMatrix(
        new double[][]{{1d, 0d, 0d, 0d}, {0d, 1d, 0d, 0d}}
    );
    Array2DRowRealMatrix Q = new Array2DRowRealMatrix(
        new double[][]{
            {Math.pow(dt, 4d) / 4d, 0d, Math.pow(dt, 3d) / 2d, 0d},
            {0d, Math.pow(dt, 4d) / 4d, 0d, Math.pow(dt, 3d) / 2d},
            {Math.pow(dt, 3d) / 2d, 0d, Math.pow(dt, 2d), 0d},
            {0d, Math.pow(dt, 3d) / 2d, 0d, Math.pow(dt, 2d)},
        }
    );

    Array2DRowRealMatrix R = new Array2DRowRealMatrix(
        new double[][]{
            {Math.pow(measurementNoise, 2d), 0d},
            {0d, Math.pow(measurementNoise, 2d)},
        }
    );

    Array2DRowRealMatrix PO = new Array2DRowRealMatrix(
        new double[][]{
            {.001, 0, 0, 0},
            {0, .001, 0, 0},
            {0, 0, .001, 0},
            {0, 0, 0, .001},
        }
    );

    ArrayRealVector x = new ArrayRealVector(new double[]{0, 0, 0, 0});

    DefaultProcessModel p = new DefaultProcessModel(A, B, Q, x, PO);
    DefaultMeasurementModel m = new DefaultMeasurementModel(H, R);

    public KalmanFilter kf = new KalmanFilter(p, m);

    public RSKalmanFilter(double m) {
        measurementNoise = m;
    }

    public Pose2d estimate(Pose2d pose, ChassisSpeeds cs) {
        double[] pos = new double[]{pose.getX(), pose.getY()};
        double[] speed = new double[]{cs.vxMetersPerSecond, cs.vyMetersPerSecond};
        ArrayRealVector csrv = new ArrayRealVector(speed);
        kf.predict(speed);
        x = (ArrayRealVector) A.operate(x).add(B.operate(csrv));
        RealVector z = H.operate(x);
        kf.correct(z);
        double ex = kf.getStateEstimation()[0], ey = kf.getStateEstimation()[1];
        return new Pose2d(ex, ey, new Rotation2d());
    }
}
