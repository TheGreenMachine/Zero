package com.team1816.lib.util.team254;

import edu.wpi.first.math.geometry.Rotation2d;

import java.text.DecimalFormat;

import static com.team1816.lib.subsystems.drive.SwerveDrive.kFrontLeft;
import static com.team1816.lib.subsystems.drive.SwerveDrive.kFrontRight;

/**
 * A drivetrain signal containing the speed and azimuth for each wheel
 *
 * @see DriveSignal
 */
public class SwerveDriveSignal extends DriveSignal {

    /**
     * Properties
     */
    public static final double[] ZERO_SPEED = new double[]{0, 0, 0, 0};
    public static final Rotation2d[] ZERO_AZIMUTH = new Rotation2d[]{
        new Rotation2d(),
        new Rotation2d(),
        new Rotation2d(),
        new Rotation2d(),
    };
    public static final Rotation2d[] X_AZIMUTH = new Rotation2d[]{
        Rotation2d.fromDegrees(45),
        Rotation2d.fromDegrees(-45),
        Rotation2d.fromDegrees(-45),
        Rotation2d.fromDegrees(45),
    };
    public static final Rotation2d[] AZIMUTH_90_DEGREES = new Rotation2d[]{
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90),
        Rotation2d.fromDegrees(90),
    };

    public static final SwerveDriveSignal NEUTRAL = new SwerveDriveSignal(
        ZERO_SPEED,
        ZERO_AZIMUTH,
        false
    );
    public static final SwerveDriveSignal BRAKE = new SwerveDriveSignal(
        ZERO_SPEED,
        X_AZIMUTH,
        false
    );

    private double[] mWheelSpeeds;
    private Rotation2d[] mWheelAzimuths; // Radians!
    private boolean mBrakeMode;

    /**
     * Instantiates a SwerveDriveSignal
     *
     * @see this#SwerveDriveSignal(double[], Rotation2d[], boolean)
     * @see DriveSignal
     */
    public SwerveDriveSignal() {
        this(ZERO_SPEED, ZERO_AZIMUTH, false);
    }

    /**
     * Instantiates a SwerveDriveSignal
     *
     * @see this#SwerveDriveSignal(double[], Rotation2d[], boolean)
     * @see DriveSignal
     */
    public SwerveDriveSignal(double left, double right) {
        super(left, right);
        mWheelSpeeds = new double[4];
        mWheelAzimuths = ZERO_AZIMUTH;
        mBrakeMode = false;
    }

    /**
     * Instantiates a SwerveDriveSignal
     *
     * @see DriveSignal
     */
    public SwerveDriveSignal(
        double[] wheelSpeeds,
        Rotation2d[] wheelAzimuths,
        boolean brakeMode
    ) {
        super(wheelSpeeds[kFrontLeft], wheelSpeeds[kFrontRight]);
        mWheelSpeeds = wheelSpeeds;
        mWheelAzimuths = wheelAzimuths;
        mBrakeMode = brakeMode;
    }

    public double[] getWheelSpeeds() {
        return mWheelSpeeds;
    }

//    public SwerveDriveSignal toVelocity() {
//        return new SwerveDriveSignal(
//            Arrays
//                .stream(this.mWheelSpeeds)
//                .map(x -> x * inchesPerSecondToTicksPer100ms(kOpenLoopMaxVelMeters))
//                .toArray(),
//            this.mWheelAzimuths,
//            this.mBrakeMode
//        );
//    }

    /**
     * Returns the wheel azimuth rotations
     *
     * @return Rotation2d[]
     */
    public Rotation2d[] getWheelAzimuths() {
        return mWheelAzimuths;
    }

    /**
     * Returns if brake mode is implemented
     *
     * @return mBrakeMode
     */
    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    /**
     * toString() print-out utility
     *
     * @return String
     */
    @Override
    public String toString() {
        String ret_val = "DriveSignal - \n";
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        for (int i = 0; i < mWheelSpeeds.length; i++) {
            ret_val +=
                "\tWheel " +
                    i +
                    ": Speed - " +
                    mWheelSpeeds[i] +
                    ", Azimuth - " +
                    fmt.format(mWheelAzimuths[i].getDegrees()) +
                    " deg\n";
        }

        return ret_val;
    }
}
