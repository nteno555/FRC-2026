package frc2025.config;

import edu.wpi.first.math.geometry.Translation2d;


public class SubsystemConstants {
    public static double flywheelS = 0.00;
    public static double flywheelV = 0.127;
    public static double flywheelA = 0.00;
    public static double flywheelP = 0.12;
    public static double flywheelI = 0.0;
    public static double flywheelD = 0.0;
    // NOT TUNED
    public static double topFlywheelGearRatio = 1.0;
    public static double bottomFlywheelGearRatio = 1.0;
    public static double topFlywheelDiameter = 0.05; // 2 inches in meters
    public static double bottomFlywheelDiameter = 0.1; // 4 inches in meters
    public static double flywheelEfficiency = 0.9;

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
    public static double hoodRotationsPerRadian = 31.7131757633; //NOT TUNED
    public static double hoodZeroOffsetRadians = 1.22; // 70 degrees

    public static double intakekS = 0.00;
    public static double intakekG = 0.2;
    public static double intakekV = 0.175;
    public static double intakekA = 0.0;
    public static double intakekP = 1.5;
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
    public static double intakeDeployPosition = -1.5;

    public static double turretkS = 0.30;
    public static double turretkV = 0.12;
    public static double turretkG = 0.0;
    public static double turretkP = 2.0;
    public static double turretkI = 0.0;
    public static double turretkD = 0.0;
    public static double turretCruiseVelocity = 60;
    public static double turretAcceleration = 150;
    public static double turretJerk = 700;
    public static double turretSupplyCurrentLimit = 40;
    public static double turretStatorCurrentLimit = 40;
    public static double turretPeakForwardTorqueCurrent = 40;
    public static double turretPeakReverseTorqueCurrent = 40;
    public static double turretTorqueNeutralDeadband = 1;
    public static double turretRotationsPerRadian = 2.556750949;
    public static double turretMinAngleRadians = -4.7123889804;
    public static double turretMaxAngleRadians = 1.5707963268;
    public static double throughBoreZero1 = 0.631;
    public static double throughBoreZero2 = 0.536;
    public static double throughBoreRatio1 = 4.90909;
    public static double throughBoreRatio2 = 4.0; // 4 motor rotations for 1 throughgbore rotation
    public static boolean hasTurret = true;

    public static double kMaxShootVelocity = 1;

    public static Translation2d shooterToCenterPose = new Translation2d(0.13, -0.07);
    public static double turretLimelightAngleOffsetRadians = Math.toRadians(180);
    public static double turretToLimelightRadius = 0.13;
    public static double turretLimelightHeight = 0.53;
    public static double turretLimelightPitch = 15.0;
    public static double turretLimelightRoll = 0.0;
}
