package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.TalonFXUtils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveModule {
    private final TalonFX m_driveMotor;
    private final TalonFX m_turningMotor;

    private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            ModuleConstants.kPModuleDriveController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

    public SwerveModule(
            int driveMotorChannel,
            int turningMotorChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed // check this out???
    ) {
        m_driveMotor = configureDriveMotor(driveMotorChannel);
        m_turningMotor = configureTurningMotor(turningMotorChannel);
    }

    private TalonFX configureDriveMotor(int channel) {
        TalonFX driveMotor = new TalonFX(channel);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.neutralDeadband = 0.001;
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveMotor.configAllSettings(driveConfig);

        driveMotor.setInverted(true);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        return driveMotor;
    }
    private TalonFX configureTurningMotor(int channel) { 
        TalonFX turningMotor = new TalonFX(channel);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.neutralDeadband = 0.001;
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        turningMotor.configAllSettings(driveConfig);

        turningMotor.setInverted(true);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        return turningMotor;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(TalonFXUtils.TalonFXSensorVelocityToMetersPerSecond(m_driveMotor.getSelectedSensorVelocity()),
                new Rotation2d(TalonFXUtils.SwerveTalonFXSensorPositionToMeters(m_turningMotor.getSelectedSensorPosition())));
    }

    public SwerveModulePosition getPosition() {
        double distance = TalonFXUtils.TalonFXSensorPositionToMeters(m_driveMotor.getSelectedSensorPosition());
        double turningDistance = TalonFXUtils.SwerveTalonFXSensorPositionToMeters(m_turningMotor.getSelectedSensorPosition());
        return new SwerveModulePosition(
                distance, new Rotation2d(turningDistance));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(TalonFXUtils.SwerveTalonFXSensorPositionToMeters(m_turningMotor.getSelectedSensorPosition())));

        final double driveOutput = m_drivePIDController.calculate(TalonFXUtils.TalonFXSensorVelocityToMetersPerSecond(m_driveMotor.getSelectedSensorVelocity()),
                state.speedMetersPerSecond);
        final double turnOutput = m_turningPIDController.calculate(TalonFXUtils.SwerveTalonFXSensorPositionToMeters(m_turningMotor.getSelectedSensorVelocity()),
                state.angle.getRadians());

        m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
        m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
    }

    public void resetEncoders() {
        m_driveMotor.setSelectedSensorPosition(0);
        m_turningMotor.setSelectedSensorPosition(0);
    }
}
