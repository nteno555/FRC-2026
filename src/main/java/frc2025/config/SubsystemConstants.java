package frc2025.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;


public class SubsystemConstants {
    public static double climberFF = 1.0;
    public static double climberkS = 0.0;
    public static double climberkV = 0.12;
    public static double climberkA = 0.0;
    public static double climberkP = 6.0;
    public static double climberkI = 0.0;
    public static double climberkD = 0.0;
    public static double climberSupplyCurrentLimit = 100;
    public static double climberStatorCurrentLimit = 200;
    public static double climbersMetersPerRotation = 0.5;
    public static double climberMotionMagicCruiseVelocity = 60;
    public static double climberMotionMagicAcceleration = 150;
    public static double climberMotionMagicJerk = 700;

    public static double flywheelkS = 0.27;
//    public static double flywheelkV = 0.127;
    public static double flywheelkV = 0.15;
    public static double flywheelkA = 0.00;
//    public static double flywheelkP = 0.3;
    public static double flywheelkP = 0.5;
    public static double flywheelkI = 0.0;
    public static double flywheelkD = 0.0;
    public static double topFlywheelGearRatio = 1.0;
    public static double bottomFlywheelGearRatio = 2.0;
    public static double topFlywheelDiameter = 0.05; // 2 inches in meters
    public static double bottomFlywheelDiameter = 0.1; // 4 inches in meters
    public static double flywheelEfficiency = 0.80;
    public static double flywheelSupplyCurrentLimit = 100;
    public static double flywheelStatorCurrentLimit = 200;

    // Hood: 50 to 85 degrees
//    public static double hoodFF = 0.29;
    public static double hoodkS = 0.29;
    public static double hoodkV = 0.12;
    public static double hoodkA = 0.00;
    public static double hoodkP = 4;
    public static double hoodkI = 0.0;
    public static double hoodkD = 0.00;
    public static double hoodSupplyCurrentLimit = 40;
    public static double hoodStatorCurrentLimit = 40;
    public static double hoodPeakForwardTorqueCurrent = 40;
    public static double hoodPeakReverseTorqueCurrent = 40;
    public static double hoodTorqueNeutralDeadband = 1;
    public static double hoodRotationsPerRadian = 31.7131757633;
    public static double hoodZeroOffsetRadians = 1.22; // 70 degrees
    public static double hoodForwardSoftLimit = 2.84;
    public static double hoodReverseSoftLimit = -0.1;
    public static double hoodRezeroVoltage = -3.5;
    public static double hoodMinAngleRotations = 0;
    public static double hoodMaxAngleRotations = 2.90;
    public static double hoodMinAngleRadians = 1.48;
    public static double hoodMaxAngleRadians = 0.87;

    public static double intakekS = 0.1;
    public static double intakekG = 0.0;
    public static double intakekV = 0.07;
    public static double intakekA = 0.0;
    public static double intakekP = 2.0;
    public static double intakekI = 0.0;
    public static double intakekD = 0.0;
    public static double intakeSupplyCurrentLimit = 100;
    public static double intakeStatorCurrentLimit = 200;
    public static double intakeMotionMagicCruiseVelocity = 120;
    public static double intakeMotionMagicAcceleration = 300;
    public static double intakeMotionMagicJerk = 1400;
    public static double intakePeakForwardTorqueCurrent = 800;
    public static double intakePeakReverseTorqueCurrent = -800;
    public static double intakeTorqueNeutralDeadband = 1;
    public static double intakeRetractPosition = 0.3;
    public static double intakeDeployPosition = 4.0;
    public static double intakeRollerVoltage = 12.0;
    public static double intakeHalfPosition = 2.0;

    public static double spindexerShootVoltage = -12.0;
    public static double towerShootVoltage = 9.0;

    // Turret Gear Ratio: 12:24 & 44:54 & 36:125
    public static double turretkS = 0.47;
    public static double turretkV = 0.153;
    public static double turretkG = 0.0;
    public static double turretkP = 4.0;
    public static double turretkI = 0.0;
    public static double turretkD = 0.0;
    public static double turretCruiseVelocity = 60;
    public static double turretAcceleration = 150;
    public static double turretJerk = 700;
    public static double turretVoltageLimit = 6;
    public static double turretSupplyCurrentLimit = 60;
    public static double turretStatorCurrentLimit = 60;
    public static double turretPeakForwardTorqueCurrent = 800;
    public static double turretPeakReverseTorqueCurrent = -800;
    public static double turretTorqueNeutralDeadband = 0;
    public static double turretZeroOffsetRadians = 0.0;
    public static double turretRotationsPerRadian = 42.0/10.0 * 48.0/26.0 * 125.0/32.0 / (2*Math.PI);
    public static double turretMinAngleRadians = -(Math.PI / 2 + 0.3);
    public static double turretMaxAngleRadians = 3*Math.PI / 2 - 0.3;
    public static double throughBoreZero1 = 0.871;
    public static double throughBoreZero2 = 0.823;
    public static double throughBoreRatio1 = 42.0 / 10.0 * 48.0 / 26.0;
    public static double throughBoreRatio2 = 42.0 / 10.0;
//    public static double throughBoreRatio2 = 4.201172;
//    public static double throughBoreRatio1 = 12.8667;
    public static double turretHomeAngleRotations = turretMinAngleRadians * turretRotationsPerRadian; // hard stop position in motor rotations
    public static boolean hasTurret = true;
    public static boolean useTurretThroughBores = true;
    public static double turretShotReadyThreshold = 0.1; // 5.73 degrees

    // 2.195068 15.061768
    // 17.197021  21.398193

    public static Transform2d robotToShooter = new Transform2d(0.09087, -0.0838, Rotation2d.kZero); // should be x = forward, y = side
    public static Transform2d shooterToCamera = new Transform2d(0.1671, 0.0, Rotation2d.kZero);
    public static double robotToTurretCameraForward = 0.0;
    public static double robotToTurretCameraRight = 0.0;
    public static double robotToTurretCameraUp = 0.5062;
    public static double robotToTurretCameraYaw = 0.0;
    public static double robotToTurretCameraPitch = 30.0;
    public static double robotToTurretCameraRoll = 0.0;
    public static boolean ignoreTurretLimelight = false;

    public static Transform2d robotToFrontCamera = new Transform2d(0.326, -0.0277, Rotation2d.kZero);
    public static double robotToFrontCameraForward = robotToFrontCamera.getX();
    public static double robotToFrontCameraRight = robotToFrontCamera.getY();
    public static double robotToFrontCameraUp = 0.1273;
    public static double robotToFrontCameraYaw = 0.0;
    public static double robotToFrontCameraPitch = 28.0;
    public static double robotToFrontCameraRoll = 180.0;

    public static Transform2d robotToBackCamera = new Transform2d(-0.326, -0.1015, Rotation2d.kZero);
    public static double robotToBackCameraForward = robotToBackCamera.getX();
    public static double robotToBackCameraRight = robotToBackCamera.getY();
    public static double robotToBackCameraUp = 0.13;
    public static double robotToBackCameraYaw = 180.0;
    public static double robotToBackCameraPitch = 28.0;
    public static double robotToBackCameraRoll = 180.0;

    public static Transform2d robotToRightCamera = new Transform2d(0.1015, 0.3274, Rotation2d.kZero);
    public static double robotToRightCameraForward = robotToRightCamera.getX();
    public static double robotToRightCameraRight = robotToRightCamera.getY();
    public static double robotToRightCameraUp = 0.1273;
    public static double robotToRightCameraYaw = -90.0;
    public static double robotToRightCameraPitch = 28.0;
    public static double robotToRightCameraRoll = 180.0;

    public static double autoAimMaxSpeed = 1.1;
    public static double shuttlingMaxSpeed = 3.5;
    public static double autoAimMaxAccel = 10.0;

    public static int climberID = 22;
    public static int flywheelMasterID = 13;
    public static int flywheelSlaveID = 14;
    public static int hoodID = 15;
    public static int indexerID = 16;
    public static int intakeDeployMasterID = 17;
    public static int intakeDeploySlaveID = 18;
    public static int intakeRollerID = 19;
    public static int towerID = 20;
    public static int turretID = 21;
    public static int turretTb1ID = 5;
    public static int turretTb2ID = 6;
}
