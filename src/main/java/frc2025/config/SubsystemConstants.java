package frc2025.config;

import edu.wpi.first.math.geometry.Translation2d;


public class SubsystemConstants {
    public static double flywheelS = 0.00;
    public static double flywheelV = 0.127;
    public static double flywheelA = 0.00;
    public static double flywheelP = 0.05;
    public static double flywheelI = 0.0;
    public static double flywheelD = 0.0;

    public static double pivotFF = 0.271;
    public static double pivotkS = 0.0;
    public static double pivotkV = 0.12;
    public static double pivotkA = 0.00;
    public static double pivotkP = 2.0;
    public static double pivotkI = 0.0;
    public static double pivotkD = 0.00;
    public static double pivotCruiseVelocity = 20;
    public static double pivotAcceleration = 40;
    public static double pivotJerk = 700;
    public static double pivotSupplyCurrentLimit = 40;
    public static double pivotStatorCurrentLimit = 40;
    public static double pivotPeakForwardTorqueCurrent = 40;
    public static double pivotPeakReverseTorqueCurrent = 40;
    public static double pivotTorqueNeutralDeadband = 1;
    public static double pivotRotationsPerDegree = 0.4558;
    public static double pivotkGFactor = 0.05;
    public static double pivotZeroOffsetDegrees = 1;

    public static double intakekS = 0.00;
    public static double intakekV = 0.175;
    public static double intakekA = 0.0;
    public static double intakekP = 2.0;
    public static double intakekI = 0.0;
    public static double intakekD = 0.0;
    public static double intakeCruiseVelocity = 10;
    public static double intakeAcceleration = 20;
    public static double intakeJerk = 700;
    public static double intakeSupplyCurrentLimit = 40;
    public static double intakeStatorCurrentLimit = 40;
    public static double intakePeakForwardTorqueCurrent = 40;
    public static double intakePeakReverseTorqueCurrent = 40;
    public static double intakeTorqueNeutralDeadband = 1;
    public static double intakeRotationsPerMeter = 0.208333333333;
    public static double intakekG = 0.17;
    public static double intakeZeroOffsetDegrees = 20;
    public static double intakeDeployPosition = -3.70;

    public static double kMaxShootVelocity = 1;

    public static Translation2d shooterToCenterPose = new Translation2d(0.0, -0.1);
}
