package frc2025.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc2025.util.InterpolatingTreeMap;


public class SubsystemConstants {
    public static double upperS = 0.32;
    public static double upperV =  0.121;
    public static double upperP = 0.5;
    public static double upperI = 0.0;
    public static double upperD =  0.0;

    public static double lowerS = 0.05;
    public static double lowerV = 0.120;
    public static double lowerA = 0.0;
    public static double lowerP = 0.03;
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

    public static double intakekS = 0.67;
    public static double intakekV = 0.67;
    public static double intakekP = 0.67;
    public static double intakekI = 0.67;
    public static double intakekD = 0.67;
    public static double intakeCruiseVelocity = 100;
    public static double intakeAcceleration = 200;
    public static double intakeJerk = 7000;
    public static double intakeSupplyCurrentLimit = 40;
    public static double intakeStatorCurrentLimit = 40;
    public static double intakePeakForwardTorqueCurrent = 40;
    public static double intakePeakReverseTorqueCurrent = 40;
    public static double intakeTorqueNeutralDeadband = 1;
    public static double intakeRotationsPerDegree = 0.208333333333;
    public static double intakekGFactor = 0.0;
    public static double intakeZeroOffsetDegrees = 20;
    public static double intakeDeployPosition = 15;

    public static double kMaxShootVelocity = 1;

    public static Translation2d shooterToCenterPose = new Translation2d(0.0, -0.2);
}
