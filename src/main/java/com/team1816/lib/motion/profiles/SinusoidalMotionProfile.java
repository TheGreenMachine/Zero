package com.team1816.lib.motion.profiles;

/**
 * This class will construct a sinusoidal motion profile.
 * Unlike a simple trapezoidal motion profile, this will allow for arbitrary n-levels of smooth continuity and only uses
 * three segmented phases which is lower than the SCurveMotionProfile
 *
 * @see MotionProfile for documentation
 */

public class SinusoidalMotionProfile extends MotionProfile {

    private Phase[] p = new Phase[3]; // sinusoidal acceleration, flat, deceleration
    private Constraints constraints;
    private State initial;
    private State target;
    private double duration;

    private double targetMaxVelocity = 0;
    private double targetMaxAcceleration = 0;

    public SinusoidalMotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
            p[i].duration = 0;
        }
        duration = 0;
        new SinusoidalMotionProfile(new Constraints(), new State(), new State());
    }

    public SinusoidalMotionProfile(Constraints c, State i, State t) {
        super();
        constraints = c;
        initial = i;
        target = t;

        double dX = t.position - i.position;
        double dV = t.velocity - i.velocity;

        double t1 = 0;
        double t2 = 0;
        double t3 = 0;

        if (dX >= 0) {
            targetMaxVelocity = c.getMaxVel();
            targetMaxAcceleration = c.getMaxAccel();
        } else {
            targetMaxVelocity = c.getMaxVel() * (-1);
            targetMaxAcceleration = c.getMaxAccel() * (-1);
        }

        if (c.getMaxAccel() == 0) {
            c.maxAccel = 1;
        }

        do {
            double cx = 0;

            t1 =
                Math.abs(
                    Math.PI / 2 * Math.abs(targetMaxVelocity - i.velocity) / c.maxAccel
                );
            t3 =
                Math.abs(
                    Math.PI / 2 * Math.abs(targetMaxVelocity - t.velocity) / c.maxAccel
                );

            // cx calculations
            cx +=
                Math.pow(targetMaxVelocity + i.velocity, 2) /
                    c.maxAccel *
                    (targetMaxVelocity - i.velocity) *
                    Math.PI /
                    4;
            cx +=
                Math.pow(targetMaxVelocity + t.velocity, 2) /
                    c.maxAccel *
                    (targetMaxVelocity - t.velocity) *
                    Math.PI /
                    4;
            cx += initial.velocity * t1;
            cx += target.velocity * t3;

            t2 += (Math.abs(dX) - cx) / c.maxVel;
        } while (t2 < 0);

        for (int x = 0; x < p.length; x++) {
            p[x] = new Phase();
        }

        p[0].duration = t1;
        p[1].duration = t2;
        p[2].duration = t3;

        duration = 0;
        for (Phase ph : p) {
            duration += Math.max(ph.duration, 0);
        }
    }

    public Constraints getConstraints() {
        return constraints;
    }

    public Phase[] getPhases() {
        return p;
    }

    public double getPosition(double t) {
        return 0;
    }

    public double getVelocity(double t) {
        double cv = 0;
        double tmp = p[0].duration;
        if (t <= tmp) {
            cv +=
                (targetMaxVelocity + initial.velocity) /
                    (-2.0) *
                    Math.cos(
                        2 *
                            (t) *
                            constraints.maxAccel /
                            (targetMaxVelocity - initial.velocity)
                    ) +
                    (targetMaxVelocity + initial.velocity) /
                        2.0;
            return cv;
        }
        cv +=
            (targetMaxVelocity + initial.velocity) /
                (-2.0) *
                Math.cos(
                    2 * (tmp) * constraints.maxAccel / (targetMaxVelocity - initial.velocity)
                ) +
                (targetMaxVelocity + initial.velocity) /
                    2.0;
        tmp += p[1].duration;
        if (t <= tmp) {
            return cv; // flat
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            cv +=
                (targetMaxVelocity + target.velocity) /
                    (2.0) *
                    Math.cos(
                        2 *
                            (tmp - t - p[2].duration) *
                            constraints.maxAccel /
                            (targetMaxVelocity - target.velocity)
                    ) +
                    (targetMaxVelocity + target.velocity) /
                        2.0;
            return cv;
        }
        return 0;
    }

    public double getAcceleration(double t) {
        double ca = 0;
        double tmp = p[0].duration;
        if (t <= tmp) {
            ca +=
                -constraints.maxAccel *
                    Math.sin(
                        2 *
                            (t) *
                            constraints.maxAccel /
                            (targetMaxVelocity - initial.velocity)
                    );
            return ca;
        }
        tmp += p[1].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            ca +=
                constraints.maxAccel *
                    Math.sin(
                        2 *
                            (tmp - t - p[2].duration) *
                            constraints.maxAccel /
                            (targetMaxVelocity - target.velocity)
                    );
            return ca;
        }
        return 0;
    }

    public double getJerk(double t) {
        double cj = 0;
        double tmp = p[0].duration;
        if (t <= tmp) {
            return (
                -getVelocity(t) *
                    Math.pow(
                        2 * constraints.maxAccel / (targetMaxVelocity - initial.velocity),
                        2
                    )
            );
        }
        tmp += p[1].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return (
                -getVelocity(t) *
                    Math.pow(
                        2 * constraints.maxAccel / (targetMaxVelocity - target.velocity),
                        2
                    )
            );
        }
        return 0;
    }

    public double getDuration() {
        return duration;
    }

    public boolean isFinished(double t) {
        return t >= duration;
    }
}
