// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class GyroConstants {
    public static final int kPigeon2IMUPort = 0; // This is correct
    public static final boolean kPigeon2IMUReversed = false;
  }

  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {
    public static final int kBackLeftDriveMotorPort = 12;
    public static final int kBackLeftTurningMotorPort = 11;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    
    public static final int kBackRightDriveMotorPort = 22;
    public static final int kBackRightTurningMotorPort = 21;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;
    
    public static final int kFrontLeftDriveMotorPort = 32;
    public static final int kFrontLeftTurningMotorPort = 31;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    
    public static final int kFrontRightDriveMotorPort = 42;
    public static final int kFrontRightTurningMotorPort = 41;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    
    // set this later

    public static final double kTrackWidth = 0.81;
    public static final double kWheelBase = 0.71;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kTalonFXCPR = 2048; // https://v5.docs.ctr-electronics.com/en/latest/ch14_MCSensor.html#sensor-resolution
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches outer diameter DETERMINE THIS

    // I made a utility class for this to make it clear what is happening
    // public static final double kDriveEncoderDistancePerPulse =
    //     // Assumes the encoders are directly mounted on the wheel shafts
    //     (kWheelDiameterMeters * Math.PI) / (double) kTalonFXCPR;

    // public static final double kTurningEncoderDistancePerPulse =
    //     // Assumes the encoders are on a 1:1 reduction with the module shaft.
    //     (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class AutonomousConstants {
    // Add these in when we need them
  }
}
