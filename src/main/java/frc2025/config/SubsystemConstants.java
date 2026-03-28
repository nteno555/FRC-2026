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

    public static double flywheelkS = 0.3;
    public static double flywheelkV = 0.123;
    public static double flywheelkA = 0.00;
    public static double flywheelkP = 0.2;
    public static double flywheelkI = 0.0;
    public static double flywheelkD = 0.0;


    // 20/4.8 = 4.167 rps/s^2 @ 10A
    // 20/0.4 = 50 rps/s^2 @ 20A
    // kA = (20A - 10A) / (50.0 - 4.167) = 10 / 45.833 = 0.218 A/(rps/s^2)

    public static double flywheelkSAmps = 10.0;
    public static double flywheelkVAmps = 0.0;
    public static double flywheelkAAmps = 0.218;
    public static double flywheelkPAmps = 5.0;
    public static double flywheelkIAmps = 0.0;
    public static double flywheelkDAmps = 0.0;

    public static double flywheelkSAmpsBangBang = 17.0;
    public static double flywheelkVAmpsBangBang = 0.0;
    public static double flywheelkAAmpsBangBang = 0.0;
    public static double flywheelkPAmpsBangBang = 0.0;
    public static double flywheelkIAmpsBangBang = 0.0;
    public static double flywheelkDAmpsBangBang = 0.0;

    public static double topFlywheelGearRatio = 1.0;
    public static double bottomFlywheelGearRatio = 2.0;
    public static double topFlywheelDiameter = 0.05; // 2 inches in meters
    public static double bottomFlywheelDiameter = 0.1; // 4 inches in meters
    public static double flywheelEfficiency = 0.80;
    public static double flywheelSupplyCurrentLimit = 200;
    public static double flywheelStatorCurrentLimit = 200;
    // to do check
    public static double towerkS = 0.37;
    public static double towerkV = 0.095;
    public static double towerkA = 0.00;
    public static double towerkP = 0.2;
    public static double towerkI = 0.0;
    public static double towerkD = 0.0;

    // Flywheel to Tower is 3 : 1
    public static double towerkSAmps = 0.0;
    public static double towerkVAmps = 0.0;
    public static double towerkAAmps = 0.00;
    public static double towerkPAmps = 5.0;
    public static double towerkIAmps = 0.0;
    public static double towerkDAmps = 0.0;
    public static double towerFlywheelRatio = 1.0;

    public static double hoodkS = 0.29;
    public static double hoodkV = 0.12;
    public static double hoodkP = 6.0;
    public static double hoodkI = 0.0;
    public static double hoodkD = 0.00;
    public static double hoodSupplyCurrentLimit = 40;
    public static double hoodStatorCurrentLimit = 40;
    public static double hoodPeakForwardTorqueCurrent = 40;
    public static double hoodPeakReverseTorqueCurrent = 40;
    public static double hoodTorqueNeutralDeadband = 1;
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
    public static double intakePeakForwardTorqueCurrent = 100;
    public static double intakePeakReverseTorqueCurrent = 100;
    public static double intakeTorqueNeutralDeadband = 1;
    public static double intakePeakForwardVoltage = 4.0;
    public static double intakePeakReverseVoltage = -4.0;
    public static double intakeRetractPosition = 0.3;
    public static double intakeDeployPosition = 4.1;
    public static double intakeRollerVoltage = 8.0;
    public static double intakeHalfPosition = 1.5;

    public static double indexerSupplyCurrentLimit = 120;
    public static double indexerStatorCurrentLimit = 120;
    public static double indexerPeakForwardTorqueCurrent = 120;
    public static double indexerPeakReverseTorqueCurrent = 120;
    public static double indexerTorqueNeutralDeadband = 1;
    public static double indexerShootVoltage = 12.0;
    public static double indexerReverseVoltage = -2.0;

    // Turret Gear Ratio: 12:24 & 44:54 & 36:125
    public static double turretkS = 0.27;
    public static double turretkV = 0.114;
    public static double turretkA = 0.001 * 12.0;
    public static double turretkP = 2.5;
    // public static double turretkP = 1.0;
    public static double turretkI = 0.0;
    public static double turretkD = 0.10;
    // public static double turretkD = 0.00;
    public static double turretCruiseVelocity = 60;
    public static double turretAcceleration = 150;
    public static double turretJerk = 700;
    public static double turretSupplyCurrentLimit = 120;
    public static double turretStatorCurrentLimit = 120;
    public static double turretPeakForwardTorqueCurrent = 120;
    public static double turretPeakReverseTorqueCurrent = -120;
    public static double turretZeroOffsetRadians = 0.0;
    public static double turretRotationsPerRadian = 42.0/11.0 * 42.0/26.0 * 110/10.0 / (2*Math.PI); //10.80
    public static double turretMinAngleRadians = -3.89;
    public static double turretMaxAngleRadians = 5.5;
    public static double throughBoreZero1 = 0.831;
    public static double throughBoreZero2 = 0.457;
    public static double throughBoreRatio1 = 42.0 / 11.0 * 42.0 / 26.0; // 6.17
    public static double throughBoreRatio2 = 42.0 / 11.0; // 3.82
    public static double turretHomeAngleRotations = turretMinAngleRadians * turretRotationsPerRadian; // hard stop position in motor rotations
    public static boolean hasTurret = true;
    public static boolean useTurretThroughBores = true;
    public static double turretShotReadyThreshold = 0.15; // 5.73 degrees
    public static boolean useTurretVisionCorrection = false;

    public static Transform2d robotToShooter = new Transform2d(0.0885, -0.1372, Rotation2d.kZero); // should be x = forward, y = side
    public static Transform2d shooterToCamera = new Transform2d(0.1671, 0.0, Rotation2d.kZero);
    public static double robotToTurretCameraForward = 0.0;
    public static double robotToTurretCameraRight = 0.0;
    public static double robotToTurretCameraUp = 0.5062;
    public static double robotToTurretCameraYaw = 0.0;
    public static double robotToTurretCameraPitch = 30.0;
    public static double robotToTurretCameraRoll = 0.0;
    public static boolean ignoreTurretLimelight = false;

    public static Transform2d robotToFrontCamera = new Transform2d(0.326, -0.1015, Rotation2d.kZero);
    public static double robotToFrontCameraForward = robotToFrontCamera.getX();
    public static double robotToFrontCameraRight = robotToFrontCamera.getY();
    public static double robotToFrontCameraUp = 0.14641;
    public static double robotToFrontCameraYaw = 0.0;
    public static double robotToFrontCameraPitch = 28.0;
    public static double robotToFrontCameraRoll = 180.0;

    public static Transform2d robotToBackCamera = new Transform2d(-0.3429, 0.064, Rotation2d.kZero);
    public static double robotToBackCameraForward = robotToBackCamera.getX();
    public static double robotToBackCameraRight = robotToBackCamera.getY();
    public static double robotToBackCameraUp = 0.1651;
    public static double robotToBackCameraYaw = 180.0;
    public static double robotToBackCameraPitch = 28.0;
    public static double robotToBackCameraRoll = 180.0;

    public static Transform2d robotToRightCamera = new Transform2d(-0.2766, 0.3429, Rotation2d.kZero);
//    public static Transform2d robotToRightCamera = new Transform2d(0.1015, 0.3274, Rotation2d.kZero);
    public static double robotToRightCameraForward = robotToRightCamera.getX();
    public static double robotToRightCameraRight = robotToRightCamera.getY();
    public static double robotToRightCameraUp = 0.2222;
    public static double robotToRightCameraYaw = -90.0;
    public static double robotToRightCameraPitch = 28.0;
    public static double robotToRightCameraRoll = 0.0;

    public static double autoAimMaxSpeed = 10.0;//2.3;
    public static double shuttlingMaxSpeed = 5.0;
    public static double autoAimMaxSpeedStrict = 10.0;//1.0;
    public static double autoAimMaxAccel = 10.0;
    public static double autoAimMaxAngularVelocity = Math.PI;
    public static double autoAimMaxAngularVelocityStrict = Math.PI/2;
    public static double shuttlingMaxAngularVelocity = Math.PI * 4;

    public static double flushFlywheelRps = 42.0;
    public static double flushHoodAngle = 0.9;

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


    /*

    ```
     */
}
