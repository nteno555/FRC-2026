package frc2025.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    // -----------------------
    // PID & Feedforward Gains
    // -----------------------
    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(2.66).withKA(0);

    private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124).withKA(0);

    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // -----------------------
    // Physical Constants
    // -----------------------
    private static final double kSlipCurrentA = 120;
    public static final double kSpeedAt12VoltsMps = 5.41;
    private static final double kCoupleRatio = 3.125;
    private static final double kDriveGearRatio = 5.902777777777778;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2.0;
    public static final double kMaxAngularVelocity = 3.14 * 2;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;
    private static final boolean kFrontLeftSteerInvert = true;
    private static final boolean kFrontRightSteerInvert = true;
    private static final boolean kBackLeftSteerInvert = true;
    private static final boolean kBackRightSteerInvert = true;

    private static final int kPigeonId = 22;
    private static final String kCANbusName = "Awesome Sauce";

    // -----------------------
    // Simulation Constants
    // -----------------------
    private static final double kSteerInertia = 0.01;
    private static final double kDriveInertia = 0.01;
    private static final double kSteerFrictionVoltage = 0.2;
    private static final double kDriveFrictionVoltage = 0.2;

    public static final LegacySwerveDrivetrainConstants DrivetrainConstants =
            new LegacySwerveDrivetrainConstants()
                    .withPigeon2Id(kPigeonId)
                    .withCANbusName(kCANbusName);

    private static final LegacySwerveModuleConstantsFactory ConstantCreator =
            new LegacySwerveModuleConstantsFactory()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withWheelRadius(kWheelRadiusInches)
                    .withSlipCurrent(kSlipCurrentA)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                    .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(kDriveFrictionVoltage)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withSteerMotorInverted(true);

    // -----------------------
    // Module IDs and Offsets
    // -----------------------
    // Front Left
    private static final int kFrontLeftDriveMotorId = 11;
    private static final int kFrontLeftSteerMotorId = 16;
    private static final int kFrontLeftEncoderId = 30;
    private static final double kFrontLeftEncoderOffset = 0.34375;
    private static final double kFrontLeftXPosInches = 10.375;
    private static final double kFrontLeftYPosInches = 10.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = 15;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 50;
    private static final double kFrontRightEncoderOffset = 0.30419921875;
    private static final double kFrontRightXPosInches = 10.375;
    private static final double kFrontRightYPosInches = -10.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = 1;
    private static final int kBackLeftSteerMotorId = 20;
    private static final int kBackLeftEncoderId = 0;
    private static final double kBackLeftEncoderOffset = -0.25048828125;
    private static final double kBackLeftXPosInches = -10.375;
    private static final double kBackLeftYPosInches = 10.375;

    // Back Right
    private static final int kBackRightDriveMotorId = 41;
    private static final int kBackRightSteerMotorId = 2;
    private static final int kBackRightEncoderId = 1;
    private static final double kBackRightEncoderOffset = 0.031494140625;
    private static final double kBackRightXPosInches = -10.375;
    private static final double kBackRightYPosInches = -10.375;

    // -----------------------
    // Legacy Swerve Modules
    // -----------------------
    public static final LegacySwerveModuleConstants FrontLeft =
            ConstantCreator.createModuleConstants(
                            kFrontLeftSteerMotorId,
                            kFrontLeftDriveMotorId,
                            kFrontLeftEncoderId,
                            kFrontLeftEncoderOffset,
                            Units.inchesToMeters(kFrontLeftXPosInches),
                            Units.inchesToMeters(kFrontLeftYPosInches),
                            kInvertLeftSide)
                    .withSteerMotorInverted(kFrontLeftSteerInvert);

    public static final LegacySwerveModuleConstants FrontRight =
            ConstantCreator.createModuleConstants(
                            kFrontRightSteerMotorId,
                            kFrontRightDriveMotorId,
                            kFrontRightEncoderId,
                            kFrontRightEncoderOffset,
                            Units.inchesToMeters(kFrontRightXPosInches),
                            Units.inchesToMeters(kFrontRightYPosInches),
                            kInvertRightSide)
                    .withSteerMotorInverted(kFrontRightSteerInvert);

    public static final LegacySwerveModuleConstants BackLeft =
            ConstantCreator.createModuleConstants(
                            kBackLeftSteerMotorId,
                            kBackLeftDriveMotorId,
                            kBackLeftEncoderId,
                            kBackLeftEncoderOffset,
                            Units.inchesToMeters(kBackLeftXPosInches),
                            Units.inchesToMeters(kBackLeftYPosInches),
                            kInvertLeftSide)
                    .withSteerMotorInverted(kBackLeftSteerInvert);

    public static final LegacySwerveModuleConstants BackRight =
            ConstantCreator.createModuleConstants(
                            kBackRightSteerMotorId,
                            kBackRightDriveMotorId,
                            kBackRightEncoderId,
                            kBackRightEncoderOffset,
                            Units.inchesToMeters(kBackRightXPosInches),
                            Units.inchesToMeters(kBackRightYPosInches),
                            kInvertRightSide)
                    .withSteerMotorInverted(kBackRightSteerInvert);
}
