package frc.robot.utils;

import frc.robot.Constants;

public class TalonFXUtils {
    public static double TalonFXSensorVelocityToMetersPerSecond(double talonFXSensorVelocity) {
        talonFXSensorVelocity /= 10;                                                     // units per every 1000ms or 1s
        
        // There are kTalonFXCPR (2048 in the case of the integrated sensor) units per 1 rotation of the wheel
        talonFXSensorVelocity /= Constants.ModuleConstants.kTalonFXCPR;                // rotations per every 1s

        // There is 2*pi*r or pi*d meters in 1 rotation
        talonFXSensorVelocity *= (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI);         

        return talonFXSensorVelocity;
    }

    public static double TalonFXSensorPositionToMeters(double talonFXSensorPosition) {
        // currently measured in units or ticks if you will. There are kTalonFXCPR units per rotation
        talonFXSensorPosition /= Constants.ModuleConstants.kTalonFXCPR; // some fraction of a rotation

        // There is 2*pi*r or pi*d meters in 1 rotation
        talonFXSensorPosition *= (Constants.ModuleConstants.kWheelDiameterMeters * Math.PI); // now in meters

        return talonFXSensorPosition;
    }

    public static double SwerveTalonFXSensorPositionToMeters(double swerveTalonFXSensorPosition) {
        // currently measured in units or ticks if you will. There are kTalonFXCPR units per rotation
        return swerveTalonFXSensorPosition * 2 * Math.PI / Constants.ModuleConstants.kTalonFXCPR;
    }

}
