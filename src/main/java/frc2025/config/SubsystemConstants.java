package frc2025.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class SubsystemConstants {
    public static final double flywheelS = 0.00;
    public static final double flywheelV = 0.13;
    public static final double flywheelA = 0.00;
    public static final double flywheelP = 0.12;
    public static final double flywheelI = 0.0;
    public static final double flywheelD = 0.0;
    // NOT TUNED
    public static final double topFlywheelGearRatio = 1.0;
    public static final double bottomFlywheelGearRatio = 1.0;
    public static final double topFlywheelDiameter = 0.05; // 2 inches in meters
    public static final double bottomFlywheelDiameter = 0.1; // 4 inches in meters
    public static final double flywheelEfficiency = 1;

    public static final double hoodFF = 0.271;
    public static final double hoodkS = 0.0;
    public static final double hoodkV = 0.12;
    public static final double hoodkA = 0.00;
    public static final double hoodkP = 1.0;
    public static final double hoodkI = 0.0;
    public static final double hoodkD = 0.00;
    public static final double hoodCruiseVelocity = 20;
    public static final double hoodAcceleration = 40;
    public static final double hoodJerk = 700;
    public static final double hoodSupplyCurrentLimit = 40;
    public static final double hoodStatorCurrentLimit = 40;
    public static final double hoodPeakForwardTorqueCurrent = 40;
    public static final double hoodPeakReverseTorqueCurrent = 40;
    public static final double hoodTorqueNeutralDeadband = 1;
    public static final double hoodRotationsPerRadian = 31.7131757633;
    public static final double hoodZeroOffsetRadians = 1.22; // 70 degrees

    public static final double intakekS = 0.00;
    public static final double intakekG = 0.2;
    public static final double intakekV = 0.175;
    public static final double intakekA = 0.0;
    public static final double intakekP = 2.5;
    public static final double intakekI = 0.0;
    public static final double intakekD = 0.0;
    public static final double intakeCruiseVelocity = 10;
    public static final double intakeAcceleration = 20;
    public static final double intakeJerk = 700;
    public static final double intakeSupplyCurrentLimit = 100;
    public static final double intakeStatorCurrentLimit = 100;
    public static final double intakePeakForwardTorqueCurrent = 100;
    public static final double intakePeakReverseTorqueCurrent = 100;
    public static final double intakeTorqueNeutralDeadband = 1;
    public static final double intakeDeployPosition = -1.5;
    public static final int intakeRollerMotorID = 0;
    public static final int intakeDeployMasterMotorID = 1;
    public static final int intakeDeploySlaveMotorID = 2;

    public static final double turretkS = 0.30;
    public static final double turretkV = 0.12;
    public static final double turretkG = 0.0;
    public static final double turretkP = 3.0;
    public static final double turretkI = 0.0;
    public static final double turretkD = 0.0;
    public static final double turretCruiseVelocity = 60;
    public static final double turretAcceleration = 150;
    public static final double turretJerk = 700;
    public static final double turretSupplyCurrentLimit = 40;
    public static final double turretStatorCurrentLimit = 40;
    public static final double turretPeakForwardTorqueCurrent = 40;
    public static final double turretPeakReverseTorqueCurrent = 40;
    public static final double turretTorqueNeutralDeadband = 1;
//    public static double final turretRotationsPerRadian = 2.556750949;
//    public static double final turretRotationsPerRadian = ((14.0 / 56.0 * (44.0 / 54.0)) / (2 * Math.PI));
    //public static double final turretRotationsPerRadian = ((56.0 / 14.0) * (54.0 / 44.0)) / (2*Math.PI);
//    public static double final turretRotationsPerRadian = 14.0/56.0 * (54.0/44.0) / (2*Math.PI);
    //public static double final turretZeroOffsetRadians = -0.13;
    public static final double turretZeroOffsetRadians = 0.0;
    public static final double turretRotationsPerRadian = 2.53;
    public static final double turretMinAngleRadians = -4.7123889804;
    public static final double turretMaxAngleRadians = 1.5707963268;
    public static final double throughBoreZero1 = 0.649;
    public static final double throughBoreZero2 = 0.555;
    public static final double throughBoreRatio1 = 4.90909;
    public static final double throughBoreRatio2 = 4.0; // 4 motor rotations for 1 throughgbore rotation
    public static final boolean hasTurret = true;

    public static final double kMaxShootVelocity = 1;


    //public static final Pose2d shooterToCenterPose = new Pose2d(0.1016, -0.1016, Rotation2d.kZero);
    public static final Pose2d shooterToCenterPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
//    public staticfinal  Translation2d shooterToCenterPose = new Translation2d(0, 0);
    public static final int shooterMasterMotorID = 3;
    public static final int shooterSlaveMotorID = 4;

    public static final double turretLimelightAngleOffsetRadians = Math.toRadians(180);
    public static final double turretToLimelightRadius = 0.15;
    public static final double turretLimelightHeight = 0.53;
    public static final double turretLimelightPitch = 15.0;
    public static final double turretLimelightRoll = 0.0;

//    public static final Pose2d shooterToCenterPose = new Pose2d(0.20, 0.1, Rotation2d.kZero);
////    public static final Translation2d shooterToCenterPose = new Translation2d(0, 0);
//    public static final double turretLimelightAngleOffsetRadians = Math.toRadians(180);
//    public static final double turretToLimelightRadius = 0.2;
//    public static final double turretLimelightHeight = 0.53;
//    public static final double turretLimelightPitch = 15.0;
//    public static final double turretLimelightRoll = 0.0;
}
