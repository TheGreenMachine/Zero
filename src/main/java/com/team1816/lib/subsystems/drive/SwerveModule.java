package com.team1816.lib.subsystems.drive;

import static com.team1816.lib.subsystems.drive.Drive.NAME;
import static com.team1816.lib.subsystems.drive.Drive.factory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1816.lib.hardware.PIDSlotConfiguration;
import com.team1816.lib.hardware.components.motor.IGreenMotor;
import com.team1816.lib.util.driveUtil.DriveConversions;
import com.team1816.lib.util.driveUtil.SwerveKinematics;
import com.team1816.season.configuration.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModule implements ISwerveModule {

    /** Components */
    private final IGreenMotor driveMotor;
    private final IGreenMotor azimuthMotor;
    public final CANCoder canCoder;

    /** State */
    public SwerveModuleState moduleState;
    public SwerveModulePosition modulePosition;
    public double driveDemand;
    public double driveActual;
    public double drivePosition;
    public double azimuthDemand;
    public double azimuthActual;
    public double motorTemp; // Drive Motor Temperature

    /** Constants */
    private final ModuleConfig mModuleConfig;
    private final int AZIMUTH_TICK_MASK;
    private final double allowableError;

    /**
     * Instantiates and configures a swerve module with a CANCoder
     * @param subsystemName name of the subsystem for yaml purposes
     * @param moduleConfig configuration of the module
     * @param canCoder attached CANCoder of the module
     * @see CANCoder
     */
    public SwerveModule(
        String subsystemName,
        ModuleConfig moduleConfig,
        CANCoder canCoder
    ) {
        mModuleConfig = moduleConfig;

        System.out.println(
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

        driveMotor.configOpenloopRamp(0.25, Constants.kCANTimeoutMs);
        azimuthMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 18, 28, 1),
            Constants.kLongCANTimeoutMs
        );

        azimuthMotor.configPeakOutputForward(.4, Constants.kLongCANTimeoutMs);
        azimuthMotor.configPeakOutputReverse(-.4, Constants.kLongCANTimeoutMs);

        azimuthMotor.setNeutralMode(NeutralMode.Brake);

        azimuthMotor.configAllowableClosedloopError(
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
     * @param desiredState desiredState
     * @param isOpenLoop are the (drive) motors in openLoop (%output) or do they use closed loop control
     * @see SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        SwerveModuleState desired_state = SwerveKinematics.optimize(
            desiredState,
            getActualState().angle
        );
        driveDemand =
            DriveConversions.metersPerSecondToTicksPer100ms(
                desired_state.speedMetersPerSecond
            );
        if (!isOpenLoop) {
            driveMotor.set(ControlMode.Velocity, driveDemand);
        } else {
            driveMotor.set(ControlMode.PercentOutput, desired_state.speedMetersPerSecond); // lying to it - speedMetersPerSecond passed in is actually percent output (1 to -1)
        }
        azimuthDemand =
            DriveConversions.convertDegreesToTicks(desired_state.angle.getDegrees()) +
            mModuleConfig.azimuthEncoderHomeOffset;
        azimuthMotor.set(ControlMode.Position, azimuthDemand);
    }

    /**
     * Updates the actual state properties of the swerve module
     * @see this#getActualState()
     * @see this#getActualPosition()
     */
    public void update() {
        driveActual =
            DriveConversions.ticksToMeters(driveMotor.getSelectedSensorVelocity(0)) * 10;
        azimuthActual =
            DriveConversions.convertTicksToDegrees(
                azimuthMotor.getSelectedSensorPosition(0) -
                mModuleConfig.azimuthEncoderHomeOffset
            );

        moduleState.speedMetersPerSecond = driveActual;
        moduleState.angle = Rotation2d.fromDegrees(azimuthActual);

        drivePosition += driveActual * Constants.kLooperDt;
        modulePosition.distanceMeters = drivePosition;
        modulePosition.angle = Rotation2d.fromDegrees(azimuthActual);

        motorTemp = driveMotor.getTemperature(); // Celsius
    }

    /**
     * Returns the actual state of the swerve module
     * @return swerve module state
     * @see SwerveModuleState
     */
    public SwerveModuleState getActualState() {
        return moduleState;
    }

    /**
     * Returns the actual position of the swerve module
     * @return swerve module position properties
     * @see SwerveModulePosition
     */
    public SwerveModulePosition getActualPosition() {
        return modulePosition;
    }

    /**
     * Returns the drive motor temperature
     * @return motorTemp
     */
    @Override
    public double getMotorTemp() {
        return motorTemp;
    }

    /**
     * Returns the module name (part of config)
     * @return moduleName
     */
    @Override
    public String getModuleName() {
        return mModuleConfig.moduleName;
    }

    /**
     * Returns the desired Azimuth position (what it is set to)
     * @return azimuthDemand
     */
    @Override
    public double getDesiredAzimuth() {
        return azimuthDemand;
    }

    /**
     * Returns the actual Azimuth position (what it actually is at)
     * @return azimuthActual
     */
    @Override
    public double getActualAzimuth() {
        return azimuthActual;
    }

    /**
     * Returns the closed loop error of the Azimuth motor (in-built)
     * @return azimuthError
     */
    @Override
    public double getAzimuthError() {
        return azimuthMotor.getClosedLoopError(0);
    }

    /**
     * Returns the desired Drive motor state (what it is set to)
     * @return driveDemand
     */
    @Override
    public double getDesiredDrive() {
        return driveDemand;
    }

    /**
     * Returns the actual Drive motor state (what it actually is at)
     * @return driveActual
     */
    @Override
    public double getActualDrive() {
        return driveActual;
    }

    /**
     * Returns the "position" of the Drive motor
     * @return drivePosition
     */
    @Override
    public double getDrivePosition() {
        return drivePosition;
    }

    /**
     * Returns the closed loop error of the Drive motor (in-built)
     * @return driveError
     */
    @Override
    public double getDriveError() {
        return driveMotor.getClosedLoopError(0);
    }

    /**
     * If there is no attached absolute CANCoder then this will "zero" / configure the Azimuth sensor on the motor to
     * its initial position
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
     * @return true if tests passed
     */
    public boolean checkSystem() {
        boolean checkDrive = true;
        double actualmaxVelTicks100ms = factory.getConstant(NAME, "maxVelTicks100ms"); // if this isn't calculated right this test will fail
        driveMotor.set(ControlMode.PercentOutput, 0.2);
        Timer.delay(1);
        if (
            Math.abs(
                driveMotor.getSelectedSensorVelocity(0) - 0.2 * actualmaxVelTicks100ms
            ) >
            actualmaxVelTicks100ms /
            50
        ) {
            checkDrive = false;
        }
        driveMotor.set(ControlMode.PercentOutput, -0.2);
        Timer.delay(1);
        if (
            Math.abs(
                driveMotor.getSelectedSensorVelocity(0) + 0.2 * actualmaxVelTicks100ms
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
            azimuthMotor.set(ControlMode.Position, setPoint);
            Timer.delay(1);
            if (
                Math.abs(azimuthMotor.getSelectedSensorPosition(0) - setPoint) >
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

        public ModuleConfig() {}

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
