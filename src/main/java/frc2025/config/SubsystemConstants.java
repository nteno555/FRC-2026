package frc2025.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;


public class SubsystemConstants {
    //Alignment to Virtual Pose
    public static double alignRotationkP = 3.0;
    public static double alignRotationkI = 0.0;
    public static double alignRotationkD = 0.0;

    public static double autoAimMaxAngularVelocity = Math.PI;

    // Following Path Command
    public static final double translationkP = 0.0;
    public static final double translationkI = 0.0;
    public static final double translationkD = 0.0;
    public static final double rotationkP = 0.0;
    public static final double rotationkI = 0.0;
    public static final double rotationkD = 0.0;

    //intake
    public static double intakekS = 0.760;
    public static double intakekG = 0.0;
    public static double intakekV = 0.112;
    public static double intakekA = 0.07;
    public static double intakekP = 6.5;
    public static double intakekI = 0.0;
    public static double intakekD = 0.0;
    public static double intakeSupplyCurrentLimit = 100;
    public static double intakeStatorCurrentLimit = 100;
    public static double intakeMotionMagicCruiseVelocity = 500;
    public static double intakeMotionMagicAcceleration = 510;
    public static double intakeMotionMagicJerk = 540;
    public static double intakePeakForwardTorqueCurrent = 100;
    public static double intakePeakReverseTorqueCurrent = 100;
    public static double intakeTorqueNeutralDeadband = 1;
    public static double intakePeakForwardVoltage = 4.0;
    public static double intakePeakReverseVoltage = -4.0;
    public static double intakeRollerVoltage = -4.0;

    public static double intakeRetractPosition = 16.0;
    public static double intakeDeployPosition = 0.0;
    public static double intakeHalfPosition = -3.0;
    public static double intakeQuarterPosition = -2.0;
    
    //flywheel
    // kV Trials: (2-0.460, 14.432), (4-0.460, 31.256), (6-0.460, 46.746), (8-0.460, 63.811), (10-0.460, 79.533)
    // kS Trials: 0.460, 
    public static final double flywheelkS = 0.340;
    public static final double flywheelkV = 0.112;
    public static final double flywheelkA = 0.00;
    public static final double flywheelkP = 0.7;
    public static final double flywheelkI = 0.0;
    public static final double flywheelkD = 0.00;

    // public static final double topFlywheelGearRatio = 1.0;
    // public static final double bottomFlywheelGearRatio = 2.0;
    // public static final double topFlywheelDiameter = 0.05; // 2 inches in meters
    // public static final double bottomFlywheelDiameter = 0.1; // 4 inches in meters
    public static final double flywheelEfficiency = 0.80;
    public static final double flywheelSupplyCurrentLimit = 100;
    public static final double flywheelStatorCurrentLimit = 200;
    public static final double flywheelTorqueCurrentMaxRPS = 70.0;

    public static final double flushWheelRps = 26.0;
    public static final double flushHoodRotations = 0.0;

    // Hood: 50 to 85 degrees
    // kS: 0.260
    // kV: (1.0, 8.8), (0.5, 2.760), (1.5, 13.316)
    // public static final double hoodFF = 0.35;
    public static final double hoodkS = 0.260;
    public static final double hoodkV = 0.094;
    public static final double hoodkA = 0.00;
    public static final double hoodkP = 4.0;
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
    public static final double hoodMaxAngleRotations = 4.549316;
    public static final double hoodMinAngleRadians = 0.96987894;
    public static final double hoodMaxAngleRadians = 0.49076156;

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

//    public static final double towerkS = 0.45;
//    public static final double towerkV = 0.095;
//    public static final double towerkA = 0.00;
//    public static final double towerkP = 0.2;
//    public static final double towerkI = 0.0;
//    public static final double towerkD = 0.0;
//
//    public static final double towerkSAmps = 0.0;
//    public static final double towerkVAmps = 0.0;
//    public static final double towerkAAmps = 0.00;
//    public static final double towerkPAmps = 5.0;
//    public static final double towerkIAmps = 0.0;
//    public static final double towerkDAmps = 0.0;
//    public static final double towerFlywheelRatio = 1.0;
//    public static final boolean hasTower = false;
//    public static final boolean needDoubleTower = false;

    // Front Cam: Forward 0 Right 0 Up 0.1 Roll 0 Pitch 18 Yaw 0
    public static final Transform2d robotToShooter = new Transform2d(0.09087, -0.0838, Rotation2d.kZero);

    public static final double robotToFrontCameraForward = 0.0;
    public static final double robotToFrontCameraRight = 0.0;
    public static final double robotToFrontCameraUp = 0.5062;
    public static final double robotToFrontCameraYaw = 0.0;
    public static final double robotToFrontCameraPitch = 30.0;
    public static final double robotToFrontCameraRoll = 0.0;

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

    // Ball Camera
    public static final Transform3d robotToBallCamera = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));


    public static final double autoAimMaxSpeed = 2.0;
    public static final double autoAimMaxAccel = 10.0;


    public static final int flywheelMasterID = 17;
    public static final int flywheelSlaveA1ID = 1;

    public static final int flywheelSlaveB1ID = 31;
    public static final int flywheelSlaveB2ID = 32;

    public static final int hoodID = 0;

    public static final int indexerID = 14;
    public static final int indexerID2 = 3;

    public static final int intakeDeployMasterID = 21;
    public static final int intakeRollerID = 7;
    public static final int intakeRollerSlaveID = 11;

}
