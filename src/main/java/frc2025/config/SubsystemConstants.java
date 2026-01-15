package frc2025.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc2025.util.InterpolatingTreeMap;


public class SubsystemConstants {
    public static double upperS = 0.32;
    public static double upperV =  0.121;
    public static double upperP = 0.5;
    public static double upperI = 0.0;
    public static double upperD =  0.0;
    public static double lowerS = 0.36;
    public static double lowerV = 0.127;
    public static double lowerP = 0.5;
    public static double lowerI = 0.0;
    public static double lowerD = 0.0;

    public static double pivotkS = 0.67;
    public static double pivotkV = 0.67;
    public static double pivotkP = 0.67;
    public static double pivotkI = 0.67;
    public static double pivotkD = 0.67;
    public static double pivotCruiseVelocity = 100;
    public static double pivotAcceleration = 200;
    public static double pivotJerk = 7000;
    public static double pivotSupplyCurrentLimit = 40;
    public static double pivotStatorCurrentLimit = 40;
    public static double pivotPeakForwardTorqueCurrent = 40;
    public static double pivotPeakReverseTorqueCurrent = 40;
    public static double pivotTorqueNeutralDeadband = 1;
    public static double pivotRotationsPerDegree = 0.208333333333;
    public static double pivotkGFactor = 0.25;
    public static double pivotZeroOffsetDegrees = 20;
    public static double pivotIntakePosition = 15;

    public static double intakeRotationsPerDegree = 0.208333333333;
    public static double intakekGFactor = 0.0;
    public static double intakeZeroOffsetDegrees = 20;
    public static double intakeDeployPosition = 15;

    public static double turretkS = 0.67;
    public static double turretkV = 0.67;
    public static double turretkP = 0.67;
    public static double turretkI = 0.67;
    public static double turretkD = 0.67;
    public static double turretkGFactor = 0.25;
    public static double turretCruiseVelocity = 100;
    public static double turretAcceleration = 200;
    public static double turretJerk = 7000;
    public static double turretSupplyCurrentLimit = 40;
    public static double turretStatorCurrentLimit = 40;
    public static double turretPeakForwardTorqueCurrent = 40;
    public static double turretPeakReverseTorqueCurrent = 40;
    public static double turretTorqueNeutralDeadband = 1;
    public static double turretRotationsPerDegree = 0.208333333333;
    public static double turretZeroOffsetDegrees = 20;


    public static double kMaxShootVelocity = 1;
}
