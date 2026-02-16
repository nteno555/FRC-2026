package frc2025.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;


public class SubsystemConstants {
    public static double flywheelS = 0.0;
    public static double flywheelV = 0.123;
    public static double flywheelA = 0.00;
    public static double flywheelP = 0.127;
    public static double flywheelI = 0.0;
    public static double flywheelD = 0.0;
    // NOT TUNED
    public static double topFlywheelGearRatio = 1.0;
    public static double bottomFlywheelGearRatio = 1.0;
    public static double topFlywheelDiameter = 0.05; // 2 inches in meters
    public static double bottomFlywheelDiameter = 0.1; // 4 inches in meters
    public static double flywheelEfficiency = 1;

    // Hood: 50 to 85 degrees
    public static double hoodFF = 0.35;
    public static double hoodkS = 0.0;
    public static double hoodkV = 0.12;
    public static double hoodkA = 0.00;
    public static double hoodkP = 2.5;
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
    public static double hoodRotationsPerRadian = 31.7131757633;
    public static double hoodZeroOffsetRadians = 1.22; // 70 degrees
    public static double hoodForwardSoftLimit = 2.84;
    public static double hoodReverseSoftLimit = 0.0;
    public static double hoodRezeroVoltage = -2.5;

    public static double intakekS = 0.00;
    public static double intakekG = 0.2;
    public static double intakekV = 0.175;
    public static double intakekA = 0.0;
    public static double intakekP = 2.5;
    public static double intakekI = 0.0;
    public static double intakekD = 0.0;
    public static double intakeCruiseVelocity = 10;
    public static double intakeAcceleration = 20;
    public static double intakeJerk = 700;
    public static double intakeSupplyCurrentLimit = 100;
    public static double intakeStatorCurrentLimit = 200;
    public static double intakePeakForwardTorqueCurrent = 100;
    public static double intakePeakReverseTorqueCurrent = -100;
    public static double intakeTorqueNeutralDeadband = 1;
    public static double intakeDeployPosition = -3.9;
    public static double intakeRollerVoltage = 8.0;

    public static double spindexerShootVoltage = -12.0;
    public static double towerShootVoltage = -12.0;

    // Turret Gear Ratio: 8:60 & 44:54
    public static double turretkS = 0.45;
    public static double turretkV = 0.128;
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
    public static double turretZeroOffsetRadians = 0.0;
    public static double turretRotationsPerRadian = 5.05539761;
    public static double turretMinAngleRadians = 0;
    public static double turretMaxAngleRadians = 2.2 * Math.PI;
    public static double throughBoreZero1 = 0.641;
    public static double throughBoreZero2 = 0.505;
    public static double throughBoreRatio1 = 9.204;
    public static double throughBoreRatio2 = 7.5;
    public static double turretHomeAngleRotations = 0.0; // hard stop position in motor rotations
    public static boolean hasTurret = false;

    // Turret Cam: Forward 0 Right 0 Up 0.39 Roll 0 Pitch 29.1 Yaw 0
    // Front Cam: Forward 0 Right 0 Up 0.1 Roll 0 Pitch 18 Yaw 0
    public static Transform2d robotToShooter = new Transform2d(0.1016, -0.1016, Rotation2d.kZero); // should be x = forward, y = side
    public static Transform2d shooterToCamera = new Transform2d(0.1705, 0.0, Rotation2d.kZero);
    public static double robotToTurretCameraForward = 0.22;
    public static double robotToTurretCameraRight = 0.1;
    public static double robotToTurretCameraUp = 0.43;
    public static double robotToTurretCameraYaw = 0.0;
    public static double robotToTurretCameraPitch = 29.1;
    public static double robotToTurretCameraRoll = 0.0;

    public static Transform2d robotToFrontCamera = new Transform2d(0.25, 0.0, Rotation2d.kZero);
    public static double robotToFrontCameraForward = 0.25;
    public static double robotToFrontCameraRight = 0.1;
    public static double robotToFrontCameraUp = 0.1;
    public static double robotToFrontCameraYaw = 0.0;
    public static double robotToFrontCameraPitch = 18.0;
    public static double robotToFrontCameraRoll = 0.0;

    public static double autoAimMaxSpeed = 2.0;
    public static double autoAimMaxAccel = 10.0;

    public static int hoodID = 27;
    public static int indexerID = 21;
    public static int towerID = 25;
    public static int intakeDeployMasterID = 15;
    public static int intakeDeploySlaveID = 16;
    public static int intakeRollerID = 42;
    public static int flywheelMasterID = 13;
    public static int flywheelSlaveID = 17;
    public static int turretID = 23;
    public static int turretTb1ID = 8;
    public static int turretTb2ID = 7;
}
