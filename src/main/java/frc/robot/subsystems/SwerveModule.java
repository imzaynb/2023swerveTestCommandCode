package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ModuleConstants;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
        m_driveMotor = new TalonFX(driveMotorChannel);
        m_turningMotor = new TalonFX(turningMotorChannel);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(),
                new Rotation2d(m_turningMotor.getSelectedSensorPosition()));
    }

    public SwerveModulePosition getPosition() {
        double distance = m_driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderDistancePerPulse;
        double turningDistance = m_turningMotor.getSelectedSensorPosition()
                * ModuleConstants.kTurningEncoderDistancePerPulse;
        return new SwerveModulePosition(
                distance, new Rotation2d(turningDistance));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_turningMotor.getSelectedSensorPosition()));

        final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity(),
                state.speedMetersPerSecond);
        final double turnOutput = m_turningPIDController.calculate(m_turningMotor.getSelectedSensorVelocity(),
                state.angle.getRadians());

        m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
        m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
    }

    public void resetEncoders() {
        m_driveMotor.setSelectedSensorPosition(0);
        m_turningMotor.setSelectedSensorPosition(0);
    }
}
