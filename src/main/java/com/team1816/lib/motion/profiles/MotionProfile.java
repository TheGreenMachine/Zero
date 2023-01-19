package com.team1816.lib.motion.profiles;

/**
 * This class provides an abstract foundation for a MotionProfile which is a simple velocity curve for finer positional control
 */
public abstract class MotionProfile {

    /**
     * Internal Definitions
     */

    public static class Constraints {

        public double maxVel, maxAccel, maxJerk;

        public Constraints() {
            maxVel = 0;
            maxAccel = 0;
            maxJerk = 0;
        }

        public Constraints(double mv, double ma, double mj) {
            maxVel = mv;
            maxAccel = ma;
            maxJerk = mj;
        }

        public double getMaxVel() {
            return maxVel;
        }

        public double getMaxAccel() {
            return maxAccel;
        }

        public double getMaxJerk() {
            return maxJerk;
        }
    }

    public static class State {

        public double position, velocity;

        public State() {
            position = 0;
            velocity = 0;
        }

        public State(double p) {
            position = p;
            velocity = 0;
        }

        public State(double p, double v) {
            position = p;
            velocity = v;
        }
    }

    public static class Phase {

        public double duration;

        public Phase() {
            duration = 0;
        }
    }

    /**
     * Properties
     */
    private Constraints constraints;
    private State initial;
    private State target;

    /**
     * Instantiates a default empty profile
     */
    public MotionProfile() {
        constraints = new Constraints();
        initial = new State();
        target = new State();
    }

    /**
     * Instantiates a Motion Profile
     *
     * @param c constraints
     * @param i initial state
     * @param t final state
     */
    public MotionProfile(Constraints c, State i, State t) {
        constraints = c;
        initial = i;
        target = t;
    }

    /**
     * Returns the position of the motion profile at a specific time
     *
     * @param t time
     * @return position
     */
    public double getPosition(double t) {
        return 0;
    }

    /**
     * Returns the velocity of the motion profile at a specific time
     *
     * @param t time
     * @return velocity
     */
    public double getVelocity(double t) {
        return 0;
    }

    /**
     * Returns the acceleration of the motion profile at a specific time
     *
     * @param t time
     * @return acceleration
     */
    public double getAcceleration(double t) {
        return 0;
    }

    /**
     * Returns the jerk of the motion profile at a specific time (only useful for sinusoidal and s-curve profiles)
     *
     * @param t time
     * @return jerk
     */
    public double getJerk(double t) {
        return 0;
    }
}
