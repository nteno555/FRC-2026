package frc2025.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;


public class SubsystemConstants {
    public static final double climberFF = 1.0;
    public static final double climberkS = 0.0;
    public static final double climberkV = 0.12;
    public static final double climberkA = 0.0;
    public static final double climberkP = 6.0;
    public static final double climberkI = 0.0;
    public static final double climberkD = 0.0;
    public static final double climberSupplyCurrentLimit = 100;
    public static final double climberStatorCurrentLimit = 200;
    public static final double climbersMetersPerRotation = 0.5;
    public static final double climberMotionMagicCruiseVelocity = 60;
    public static final double climberMotionMagicAcceleration = 150;
    public static final double climberMotionMagicJerk = 700;
    

    public static final double flywheelkS = 0.0;
    public static final double flywheelkV = 0.123;
    public static final double flywheelkA = 0.00;
    public static final double flywheelkP = 0.127;
    public static final double flywheelkI = 0.0;
    public static final double flywheelkD = 0.0;
    public static final double topFlywheelGearRatio = 1.0;
    public static final double bottomFlywheelGearRatio = 2.0;
    public static final double topFlywheelDiameter = 0.05; // 2 inches in meters
    public static final double bottomFlywheelDiameter = 0.1; // 4 inches in meters
    public static final double flywheelEfficiency = 0.80;
    public static final double flywheelSupplyCurrentLimit = 100;
    public static final double flywheelStatorCurrentLimit = 200;

    // Hood: 50 to 85 degrees
    public static final double hoodFF = 0.35;
    public static final double hoodkS = 0.0;
    public static final double hoodkV = 0.12;
    public static final double hoodkA = 0.00;
    public static final double hoodkP = 2.5;
    public static final double hoodkI = 0.0;
    public static final double hoodkD = 0.00;
    public static final double hoodSupplyCurrentLimit = 40;
    public static final double hoodStatorCurrentLimit = 40;
    public static final double hoodPeakForwardTorqueCurrent = 40;
    public static final double hoodPeakReverseTorqueCurrent = 40;
    public static final double hoodTorqueNeutralDeadband = 1;
    public static final double hoodRotationsPerRadian = 31.7131757633;
    public static final double hoodZeroOffsetRadians = 1.22; // 70 degrees
    public static final double hoodForwardSoftLimit = 2.84;
    public static final double hoodReverseSoftLimit = 0.0;
    public static final double hoodRezeroVoltage = -3.5;
    public static final double hoodMinAngleRotations = 0;
    public static final double hoodMaxAngleRotations = 2.84;
    public static final double hoodMinAngleRadians = 1.48;
    public static final double hoodMaxAngleRadians = 0.87;

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
    public static final double intakeStatorCurrentLimit = 200;
    public static final double intakePeakForwardTorqueCurrent = 100;
    public static final double intakePeakReverseTorqueCurrent = -100;
    public static final double intakeTorqueNeutralDeadband = 1;
    public static final double intakeDeployPosition = -3.9;
    public static final double intakeRollerVoltage = 8.0;

    public static final double indexerSupplyCurrentLimit = 120;
    public static final double indexerStatorCurrentLimit = 120;
    public static final double indexerPeakForwardTorqueCurrent = 120;
    public static final double indexerPeakReverseTorqueCurrent = 120;
    public static final double indexerTorqueNeutralDeadband = 1;
    public static final double indexerShootVoltage = -12.0;
    public static final double indexerReverseVoltage = 2.0;
    public static final boolean needDoubleIndexer = false;

    public static final double spindexerShootVoltage = -12.0;
    public static final double towerShootVoltage = -12.0;

    public static final double towerkS = 0.45;
    public static final double towerkV = 0.095;
    public static final double towerkA = 0.00;
    public static final double towerkP = 0.2;
    public static final double towerkI = 0.0;
    public static final double towerkD = 0.0;

    public static final double towerkSAmps = 0.0;
    public static final double towerkVAmps = 0.0;
    public static final double towerkAAmps = 0.00;
    public static final double towerkPAmps = 5.0;
    public static final double towerkIAmps = 0.0;
    public static final double towerkDAmps = 0.0;
    public static final double towerFlywheelRatio = 1.0;
    public static final boolean hasTower = false;
    public static final boolean needDoubleTower = false;

    // Turret Gear Ratio: 8:60 & 44:54
    public static final double turretkS = 0.45;
    public static final double turretkV = 0.128;
    public static final double turretkG = 0.0;
    public static final double turretkP = 2.0;
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
    public static final double turretZeroOffsetRadians = 0.0;
    public static final double turretRotationsPerRadian = 5.05539761;
    public static final double turretMinAngleRadians = 0;
    public static final double turretMaxAngleRadians = 2.2 * Math.PI;
    public static final double throughBoreZero1 = 0.641;
    public static final double throughBoreZero2 = 0.505;
    public static final double throughBoreRatio1 = 9.204;
    public static final double throughBoreRatio2 = 7.5;
    public static final double turretHomeAngleRotations = 0.0; // hard stop position in motor rotations
    public static final boolean hasTurret = true;

    // Turret Cam: Forward 0 Right 0 Up 0.39 Roll 0 Pitch 29.1 Yaw 0
    // Front Cam: Forward 0 Right 0 Up 0.1 Roll 0 Pitch 18 Yaw 0
    public static final Transform2d robotToShooter = new Transform2d(0.09087, -0.0838, Rotation2d.kZero); // should be x = forward, y = side
    public static final Transform2d shooterToCamera = new Transform2d(0.1671, 0.0, Rotation2d.kZero);
    public static final double robotToTurretCameraForward = 0.0;
    public static final double robotToTurretCameraRight = 0.0;
    public static final double robotToTurretCameraUp = 0.5062;
    public static final double robotToTurretCameraYaw = 0.0;
    public static final double robotToTurretCameraPitch = 30.0;
    public static final double robotToTurretCameraRoll = 0.0;

    public static final Transform2d robotToLeftCamera = new Transform2d(-0.326, -0.0277, Rotation2d.kZero);
    public static final double robotToLeftCameraForward = robotToLeftCamera.getX();
    public static final double robotToLeftCameraRight = robotToLeftCamera.getY();
    public static final double robotToLeftCameraUp = 0.1273;
    public static final double robotToLeftCameraYaw = 0.0;
    public static final double robotToLeftCameraPitch = 28.0;
    public static final double robotToLeftCameraRoll = 0.0;

    public static final Transform2d robotToRightCamera = new Transform2d(0.326, 0.1015, Rotation2d.kZero);
    public static final double robotToRightCameraForward = robotToRightCamera.getX();
    public static final double robotToRightCameraRight = robotToRightCamera.getY();
    public static final double robotToRightCameraUp = 0.13;
    public static final double robotToRightCameraYaw = 0.0;
    public static final double robotToRightCameraPitch = 28.0;
    public static final double robotToRightCameraRoll = 0.0;

    public static final Transform2d robotToBackCamera = new Transform2d(0.3274, 0.1015, Rotation2d.kZero);
    public static final double robotToBackCameraForward = robotToBackCamera.getX();
    public static final double robotToBackCameraRight = robotToBackCamera.getY();
    public static final double robotToBackCameraUp = 0.1273;
    public static final double robotToBackCameraYaw = 0.0;
    public static final double robotToBackCameraPitch = 18.0; // to be updated
    public static final double robotToBackCameraRoll = 0.0;

    public static final double photonCameraHeight = 0.381;
    public static final Transform3d robotToPhotonCamera = new Transform3d(new Translation3d(0.34163, -0.127, photonCameraHeight), new Rotation3d(0, 0, 0));
    public static final Transform2d robotToPhotonCamera2d = new Transform2d(new Translation2d(0.34163, -0.127), new Rotation2d());


    public static final double autoAimMaxSpeed = 2.0;
    public static final double autoAimMaxAccel = 10.0;

    public static final int climberID = 19;
    public static final int flywheelMasterID = 13;
    public static final int flywheelSlaveID = 17;
    public static final int hoodID = 27;
    public static final int indexerID = 21;
    public static final int indexerID2 = 21;
    public static final int intakeDeployMasterID = 15;
    public static final int intakeDeploySlaveID = 16;
    public static final int intakeRollerID = 42;
    public static final int towerID = 25;
    public static final int towerID2 = 25;
    public static final int turretID = 23;
    public static final int turretTb1ID = 8;
    public static final int turretTb2ID = 7;
}
