package com.team1816.lib.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.season.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * An enhanced motor checking utility
 */
public class EnhancedMotorChecker {

    /**
     * Configuration
     */
    public static class CheckerConfig {

        public double mCurrentFloor = 5;
        public double mRPMFloor = 2000;

        public double mCurrentEpsilon = 5.0;
        public double mRPMEpsilon = 500;
        public DoubleSupplier mRPMSupplier = null;

        public double mRunTimeSec = 4.0;
        public double mWaitTimeSec = 2.0;
        public double mRunOutputPercentage = 0.2;

        public static CheckerConfig getForSubsystemMotor(
            Subsystem subsystem,
            IGreenMotor motor
        ) {
            var name = subsystem.getSubsystemName();
            var factory = Robot.getFactory();
            return new EnhancedMotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = factory.getConstant(name, "currentFloorCheck");
                    mRPMFloor = factory.getConstant(name, "rpmFloorCheck");
                    mCurrentEpsilon = factory.getConstant(name, "currentEpsilonCheck");
                    mRPMEpsilon = factory.getConstant(name, "rpmEpsilonCheck");
                    mRPMSupplier = () -> motor.getSelectedSensorVelocity(0);
                }
            };
        }
    }

    /**
     * Checks a certain motor
     *
     * @param subsystem    Subsystem that the motor belongs to
     * @param motorToCheck IGreenMotor to check
     * @return true if tests passed
     */
    public static boolean checkMotor(Subsystem subsystem, IGreenMotor... motorToCheck) {
        // Note: We've only checked one motor at a time - the checkerConfig is only for the first motor but is used for all motors!
        CheckerConfig checkerConfig = CheckerConfig.getForSubsystemMotor(
            subsystem,
            motorToCheck[0]
        );
        boolean failure = false;
        System.out.println("////////////////////////////////////////////////");
        System.out.println(
            "Checking subsystem " +
                subsystem.getClass() +
                " for " +
                motorToCheck.length +
                " motors."
        );

        List<Double> currents = new ArrayList<>();
        List<Double> rpms = new ArrayList<>();
        List<ControlMode> storedControlModes = new ArrayList<>();

        // Record previous configuration for all motors.
        for (IGreenMotor motor : motorToCheck) {
            if (motor.getDeviceID() < 0) continue;

            storedControlModes.add(motor.getControlMode());

            // Now set to disabled.
            motor.set(ControlMode.PercentOutput, 0.0);
        }

        for (IGreenMotor motor : motorToCheck) {
            System.out.println("Checking: " + motor.getName());

            if (motor.getDeviceID() < 0) {
                System.out.println("Motor Disabled, Checks Skipped!!");
                continue;
            }

            motor.set(ControlMode.PercentOutput, checkerConfig.mRunOutputPercentage);
            Timer.delay(checkerConfig.mRunTimeSec);

            // Now poll the interesting information.
            double current = MotorUtil.getSupplyCurrent(motor);
            currents.add(current);
            System.out.print("Current: " + current);

            double rpm = Double.NaN;
            if (checkerConfig.mRPMSupplier != null) {
                rpm = checkerConfig.mRPMSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.print(" RPM: " + rpm);
            }
            System.out.print('\n');

            motor.set(ControlMode.PercentOutput, 0.0);

            // And perform checks.
            if (current < checkerConfig.mCurrentFloor) {
                DriverStation.reportError(
                    motor.getName() +
                        " has failed current floor check vs " +
                        checkerConfig.mCurrentFloor +
                        "!!!!!!!!!!!!",
                    false
                );
                failure = true;
            }
            if (checkerConfig.mRPMSupplier != null) {
                if (rpm < checkerConfig.mRPMFloor) {
                    DriverStation.reportError(
                        motor.getName() +
                            " has failed rpm floor check vs " +
                            checkerConfig.mRPMFloor +
                            "!!!!!!!!!!!!!",
                        false
                    );
                    failure = true;
                }
            }

            Timer.delay(checkerConfig.mWaitTimeSec);
        }

        // Now run aggregate checks.

        if (currents.size() > 0) {
            Double average = currents
                .stream()
                .mapToDouble(val -> val)
                .average()
                .getAsDouble();

            if (!Util.allCloseTo(currents, average, checkerConfig.mCurrentEpsilon)) {
                DriverStation.reportError("Currents varied!!!!!!!!!!!", false);
                failure = true;
            }
        }

        if (rpms.size() > 0) {
            Double average = rpms
                .stream()
                .mapToDouble(val -> val)
                .average()
                .getAsDouble();

            if (!Util.allCloseTo(rpms, average, checkerConfig.mRPMEpsilon)) {
                DriverStation.reportError("RPMs varied!!!!!!!!", false);
                failure = true;
            }
        }

        // Restore Talon configurations
        for (int i = 0; i < motorToCheck.length; ++i) {
            IGreenMotor motor = motorToCheck[i];
            if (motor.getDeviceID() >= 0) {
                motor.set(storedControlModes.get(i), 0);
            }
        }

        return !failure;
    }
}
