package com.team1816.lib.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.hardware.components.motor.LazyTalonSRXDev;
import com.team1816.lib.hardware.components.motor.configurations.GreenControlMode;
import com.team1816.lib.util.driveUtil.DriveConversions;
import com.team1816.lib.util.driveUtil.SwerveKinematics;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.season.Robot;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import static com.team1816.lib.subsystems.drive.Drive.NAME;
import static com.team1816.lib.subsystems.drive.Drive.factory;

public class SwerveModule implements ISwerveModule {

    /**
     * Components
     */
    private final IGreenMotor driveMotor;
    private final IGreenMotor azimuthMotor;
    public final CANCoder canCoder;

    /**
     * State
     */
    public SwerveModuleState moduleState;
    public SwerveModulePosition modulePosition;
    public double driveDemandMPS;
    public double driveActualMPS;
    public double drivePosition;
    public double azimuthDemandDeg;
    public double azimuthActualDeg;
    public double motorTemp; // Drive Motor Temperature

    /**
     * Constants
     */
    private final ModuleConfig mModuleConfig;
    private final int AZIMUTH_TICK_MASK;
    private final double allowableError;

    /**
     * Instantiates and configures a swerve module with a CANCoder
     *
     * @param subsystemName name of the subsystem for yaml purposes
     * @param moduleConfig  configuration of the module
     * @param canCoder      attached CANCoder of the module
     * @see CANCoder
     */
    public SwerveModule(
        String subsystemName,
        ModuleConfig moduleConfig,
        CANCoder canCoder
    ) {
        mModuleConfig = moduleConfig;

        GreenLogger.log(
            "Configuring Swerve Module " +
                mModuleConfig.moduleName +
                " on subsystem " +
                subsystemName
        );

        AZIMUTH_TICK_MASK = (int) factory.getConstant(NAME, "azimuthEncPPR", 4096) - 1; // was 0xFFF

        /* Drive Motor Config */
        driveMotor =
            factory.getMotor(
                subsystemName,
                mModuleConfig.driveMotorName,
                factory.getSubsystem(subsystemName).swerveModules.drivePID,
                -1
            );

        /* Azimuth (Angle) Motor Config */
        azimuthMotor =
            factory.getMotor(
                subsystemName,
                mModuleConfig.azimuthMotorName,
                factory.getSubsystem(subsystemName).swerveModules.azimuthPID,
                canCoder == null ? -1 : canCoder.getDeviceID()
            );

        driveMotor.configOpenLoopRampRate(factory.getConstant("drivetrain", "openLoopRampRate", 0), Constants.kCANTimeoutMs);
        azimuthMotor.configCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 18, 28, 1),
            Constants.kLongCANTimeoutMs
        );

        azimuthMotor.config_PeakOutputForward(.4, Constants.kLongCANTimeoutMs);
        azimuthMotor.config_PeakOutputReverse(-.4, Constants.kLongCANTimeoutMs);

        azimuthMotor.setNeutralMode(NeutralMode.Brake);

        azimuthMotor.configAllowableErrorClosedLoop(
            0,
            mModuleConfig.azimuthPid.allowableError,
            Constants.kLongCANTimeoutMs
        );

        allowableError = 5; // TODO this is a dummy value for checkSystem
        drivePosition = 0;

        moduleState = new SwerveModuleState();
        modulePosition = new SwerveModulePosition();

        /* Angle Encoder Config */
        this.canCoder = canCoder;
    }

    /**
     * Sets the desired state of the swerve module
     *
     * @param desiredState desiredState
     * @param isOpenLoop   are the (drive) motors in openLoop (%output) or do they use closed loop control
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState desired_state = SwerveKinematics.optimize(
            desiredState,
            getActualState().angle
        );
        driveDemandMPS = desired_state.speedMetersPerSecond;
        double driveDemandTP100MS =
            DriveConversions.metersPerSecondToTicksPer100ms(
                desired_state.speedMetersPerSecond
            );
        azimuthDemandDeg = desired_state.angle.getDegrees();
        double azimuthDemandPos =
            DriveConversions.convertDegreesToTicks(desired_state.angle.getDegrees()) +
                mModuleConfig.azimuthEncoderHomeOffset;

        if (!isOpenLoop) {
            driveMotor.set(GreenControlMode.VELOCITY_CONTROL, driveDemandTP100MS);
        } else {
            driveDemandMPS *= Drive.kMaxVelOpenLoopMeters;
            driveMotor.set(GreenControlMode.PERCENT_OUTPUT, desired_state.speedMetersPerSecond); // lying to it - speedMetersPerSecond passed in is actually percent output (1 to -1)
        }
        azimuthMotor.set(GreenControlMode.POSITION_CONTROL, azimuthDemandPos);
    }

    /**
     * Updates the actual state properties of the swerve module
     *
     * @see this#getActualState()
     * @see this#getActualPosition()
     */
    public void update() {
        driveActualMPS =
            DriveConversions.ticksToMeters(driveMotor.getSensorVelocity(0)) * 10;
        azimuthActualDeg =
            DriveConversions.convertTicksToDegrees(
                azimuthMotor.getSensorPosition(0) -
                    mModuleConfig.azimuthEncoderHomeOffset
            );

        moduleState.speedMetersPerSecond = driveActualMPS;
        moduleState.angle = Rotation2d.fromDegrees(azimuthActualDeg);

        drivePosition += driveActualMPS * Robot.looperDt / 1000;
        modulePosition.distanceMeters = drivePosition;
        modulePosition.angle = Rotation2d.fromDegrees(azimuthActualDeg);

        motorTemp = driveMotor.getMotorTemperature(); // Celsius
    }

    /**
     * Returns the actual state of the swerve module
     *
     * @return swerve module state
     * @see SwerveModuleState
     */
    public SwerveModuleState getActualState() {
        return moduleState;
    }

    /**
     * Returns the actual position of the swerve module
     *
     * @return swerve module position properties
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getActualPosition() {
        return modulePosition;
    }

    /**
     * Returns the drive motor temperature
     *
     * @return motorTemp
     */
    @Override
    public double getMotorTemp() {
        return motorTemp;
    }

    /**
     * Returns the module name (part of config)
     *
     * @return moduleName
     */
    @Override
    public String getModuleName() {
        return mModuleConfig.moduleName;
    }

    /**
     * Returns the desired Azimuth position (what it is set to)
     *
     * @return azimuthDemand
     */
    @Override
    public double getDesiredAzimuth() {
        return azimuthDemandDeg;
    }

    /**
     * Returns the actual Azimuth position (what it actually is at)
     *
     * @return azimuthActual
     */
    @Override
    public double getActualAzimuth() {
        return azimuthActualDeg;
    }

    /**
     * Returns the closed loop error of the Azimuth motor (in-built)
     *
     * @return azimuthError
     */
    @Override
    public double getAzimuthError() {
        return azimuthMotor.getClosedLoopError();
    }

    /**
     * Returns the desired Drive motor state (what it is set to)
     *
     * @return driveDemand
     */
    @Override
    public double getDesiredDrive() {
        return driveDemandMPS;
    }

    /**
     * Returns the actual Drive motor state (what it actually is at)
     *
     * @return driveActual
     */
    @Override
    public double getActualDrive() {
        return driveActualMPS;
    }

    /**
     * Returns the "position" of the Drive motor
     *
     * @return drivePosition
     */
    @Override
    public double getDrivePosition() {
        return drivePosition;
    }

    /**
     * Returns the closed loop error of the Drive motor (in-built)
     *
     * @return driveError
     */
    @Override
    public double getDriveError() {
        return driveMotor.getClosedLoopError();
    }

    /**
     * If there is no attached absolute CANCoder then this will "zero" / configure the Azimuth sensor on the motor to
     * its initial position
     *
     * @see com.team1816.lib.hardware.components.motor.IMotorSensor
     */
    public void zeroAzimuthSensor() {
        if (azimuthMotor instanceof TalonSRX && canCoder == null) {
            var sensors = ((TalonSRX) azimuthMotor).getSensorCollection();
            sensors.setQuadraturePosition(
                sensors.getPulseWidthPosition() % AZIMUTH_TICK_MASK,
                Constants.kLongCANTimeoutMs
            );
        }
    }

    /**
     * Tests the Swerve Module based on its ability to move back and forth and rotate
     *
     * @return true if tests passed
     */
    public boolean checkSystem() {
        boolean checkDrive = true;
        double actualmaxVelTicks100ms = factory.getConstant(NAME, "maxVelTicks100ms"); // if this isn't calculated right this test will fail
        driveMotor.set(GreenControlMode.PERCENT_OUTPUT, 0.2);
        Timer.delay(1);
        if (
            Math.abs(
                driveMotor.getSensorVelocity(0) - 0.2 * actualmaxVelTicks100ms
            ) >
                actualmaxVelTicks100ms /
                    50
        ) {
            checkDrive = false;
        }
        driveMotor.set(GreenControlMode.PERCENT_OUTPUT, -0.2);
        Timer.delay(1);
        if (
            Math.abs(
                driveMotor.getSensorVelocity(0) + 0.2 * actualmaxVelTicks100ms
            ) >
                actualmaxVelTicks100ms /
                    50
        ) {
            checkDrive = false;
        }

        boolean checkAzimuth = true;
        double setPoint = mModuleConfig.azimuthEncoderHomeOffset;
        Timer.delay(1);
        for (int i = 0; i < 4; i++) {
            azimuthMotor.set(GreenControlMode.POSITION_CONTROL, setPoint);
            Timer.delay(1);
            if (
                Math.abs(azimuthMotor.getSensorPosition(0) - setPoint) >
                    allowableError
            ) {
                checkAzimuth = false;
                break;
            }
            setPoint += DriveConversions.convertRadiansToTicks(Math.PI / 2);
        }

        return checkDrive && checkAzimuth;
    }

    /**
     * toString()
     *
     * @return information on the SwerveModule
     */
    @Override
    public String toString() {
        return (
            "SwerveModule{ " +
                mModuleConfig.driveMotorName +
                " id: " +
                driveMotor.getDeviceID() +
                "  " +
                mModuleConfig.azimuthMotorName +
                " id: " +
                azimuthMotor.getDeviceID() +
                " offset: " +
                mModuleConfig.azimuthEncoderHomeOffset
        );
    }

    /**
     * Lightweight module configuration class for the entire swerve module including offsets and PID
     */
    public static class ModuleConfig {

        public ModuleConfig() {
        }

        public String moduleName = "Name";
        public String driveMotorName = "";
        public String azimuthMotorName = "";

        public PIDSlotConfiguration azimuthPid;
        public PIDSlotConfiguration drivePid;

        // constants defined for each swerve module
        public double azimuthEncoderHomeOffset;
        public static final int kAzimuthPPR = (int) factory.getConstant(
            "drive",
            "azimuthEncPPR",
            4096
        );
    }
}
