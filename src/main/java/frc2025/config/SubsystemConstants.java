package frc2025.config;

import edu.wpi.first.math.geometry.Translation2d;


public class SubsystemConstants {
    public static double flywheelS = 0.00;
    public static double flywheelV = 0.13;
    public static double flywheelA = 0.00;
    public static double flywheelP = 0.12;
    public static double flywheelI = 0.0;
    public static double flywheelD = 0.0;

    public static double hoodFF = 0.271;
    public static double hoodkS = 0.0;
    public static double hoodkV = 0.12;
    public static double hoodkA = 0.00;
    public static double hoodkP = 2.0;
    public static double hoodkI = 0.0;
    public static double hoodkD = 0.00;
    public static double hoodCruiseVelocity = 20;
    public static double hoodAcceleration = 40;
    public static double hoodJerk = 700;
    public static double hoodSupplyCurrentLimit = 40;
    public static double hoodStatorCurrentLimit = 40;
    public static double hoodPeakForwardTorqueCurrent = 40;
    public static double hoodPeakReverseTorqueCurrent = 40;
    public static double hoodTorqueNeutralDeadband = 1;
    public static double hoodRotationsPerDegree = 0.4558;
    public static double hoodkGFactor = 0.05;
    public static double hoodZeroOffsetDegrees = 1;

    public static double intakekS = 0.00;
    public static double intakekG = 0.17;
    public static double intakekV = 0.175;
    public static double intakekA = 0.0;
    public static double intakekP = 1.0;
    public static double intakekI = 0.0;
    public static double intakekD = 0.0;
    public static double intakeCruiseVelocity = 10;
    public static double intakeAcceleration = 20;
    public static double intakeJerk = 700;
    public static double intakeSupplyCurrentLimit = 100;
    public static double intakeStatorCurrentLimit = 100;
    public static double intakePeakForwardTorqueCurrent = 100;
    public static double intakePeakReverseTorqueCurrent = 100;
    public static double intakeTorqueNeutralDeadband = 1;
    public static double intakeDeployPosition = -3.70;

    public static double turretkS = 0.67;
    public static double turretkV = 0.67;
    public static double turretkG = 0.00;
    public static double turretkP = 0.67;
    public static double turretkI = 0.67;
    public static double turretkD = 0.67;
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
    public static double turretRotationsPerRadian = turretRotationsPerDegree * (180/Math.PI);
    public static double turretMinAngleRadians = 0;
    public static double turretMaxAngleRadians = 2 * Math.PI * 1.1;
    public static double throughBoreZero1 = 0.879;
    public static double throughBoreZero2 = 0.205;
    public static double throughBoreRatio1 = 3.7058823529;
    public static double throughBoreRatio2 = 3.0;
    public static boolean hasTurret = true;

    public static double kMaxShootVelocity = 1;

    public static Translation2d shooterToCenterPose = new Translation2d(0.0, -0.1);
    public static double turretLimelightAngleOffsetRadians = Math.toRadians(180);
    public static double turretToLimelightRadius = 0.1;
    public static double turretLimelightHeight = 0.4;
    public static double turretLimelightPitch = 15.0;
    public static double turretLimelightRoll = 0.0;
}
