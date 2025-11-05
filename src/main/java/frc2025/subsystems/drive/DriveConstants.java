package frc2025.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
	// Both sets of gains need to be tuned to your individual robot.

	// The steer motor uses any SwerveModule.SteerRequestType control request with the
	// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
	// VALUES FROM SYSID
	private static final Slot0Configs steerGains = new Slot0Configs()
			.withKP(100).withKI(0).withKD(0)
			.withKS(0).withKV(0.2).withKA(0);
	/*
	private static final Slot0Configs steerGains = new Slot0Configs()
			.withKP(100).withKI(0).withKD(0)
			.withKS(0).withKV(0.2).withKA(0);
	 */
	// When using closed-loop control, the drive motor uses the control
	// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
	private static final Slot0Configs driveGains = new Slot0Configs()
			.withKP(0.11).withKI(0).withKD(0)
			.withKS(0.21065).withKV(0.12188).withKA(0.01234);

	// The closed-loop output type to use for the steer motors;
	// This affects the PID/FF gains for the steer motors
	private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
	// The closed-loop output type to use for the drive motors;
	// This affects the PID/FF gains for the drive motors
	private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

	// The stator current at which the wheels start to slip;
	// This needs to be tuned to your individual robot
	private static final double kSlipCurrentA = 120;

	// Theoretical free speed (m/s) at 12v applied output;
	// This needs to be tuned to your individual robot
	public static final double kSpeedAt12VoltsMps = 5.44;

	// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
	// This may need to be tuned to your individual robot
	private static final double kCoupleRatio = 3.125;

	private static final double kDriveGearRatio = 5.902777777777778;
	private static final double kSteerGearRatio = 18.75;
	//wheel radius (meters) = gyro delta (radians) * drive base radius (meters) / wheel position delta (radians)
	//private static final double kWheelRadiusInches = 1.989436788649 * 0.93 * 1.04687;
	private static final double kWheelRadiusInches = 1.957987;

	private static final boolean kSteerMotorReversed = true;
	private static final boolean kInvertLeftSide = false;
	private static final boolean kInvertRightSide = true;

	private static final String kCANbusName = "Awesome Sauce";
	private static final int kPigeonId = 22;

	// These are only used for simulation
	private static final double kSteerInertia = 0.00001;
	private static final double kDriveInertia = 0.001;
	// Simulated voltage necessary to overcome friction
	private static final double kSteerFrictionVoltage = 0.25;
	private static final double kDriveFrictionVoltage = 0.25;

	public static final LegacySwerveDrivetrainConstants DrivetrainConstants = new LegacySwerveDrivetrainConstants()
			.withPigeon2Id(kPigeonId)
			.withCANbusName(kCANbusName);

	private static final LegacySwerveModuleConstantsFactory ConstantCreator = new LegacySwerveModuleConstantsFactory()
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
			.withSteerMotorInverted(kSteerMotorReversed);

	public static final double kMaxAngularVelocity = 3.14 * 2;


	// Front Left
	private static final int kFrontLeftDriveMotorId = 1;
	private static final int kFrontLeftSteerMotorId = 2;
	private static final int kFrontLeftEncoderId = 5;
	//private static final double kFrontLeftEncoderOffset = -0.367431640625;
	private static final double kFrontLeftEncoderOffset = 0.337890625;
	private static final boolean kFrontLeftSteerInvert = true;

	private static final double kFrontLeftXPosInches = 10.625;
	private static final double kFrontLeftYPosInches = 10.625;

	// Front Right
	private static final int kFrontRightDriveMotorId = 3;
	private static final int kFrontRightSteerMotorId = 16;
	private static final int kFrontRightEncoderId = 8;
	private static final double kFrontRightEncoderOffset = 0.37060546875;
	private static final boolean kFrontRightSteerInvert = true;

	private static final double kFrontRightXPosInches = 10.625;
	private static final double kFrontRightYPosInches = -10.625;

	// Back Left
	private static final int kBackLeftDriveMotorId = 7;
	private static final int kBackLeftSteerMotorId = 11;
	private static final int kBackLeftEncoderId = 9;
	private static final double kBackLeftEncoderOffset = 0.166015625;
	private static final boolean kBackLeftSteerInvert = true;

	private static final double kBackLeftXPosInches = -10.625;
	private static final double kBackLeftYPosInches = 10.625;

	// Back Right
	private static final int kBackRightDriveMotorId = 14;
	private static final int kBackRightSteerMotorId = 4;
	private static final int kBackRightEncoderId = 6;
	private static final double kBackRightEncoderOffset = 0.013427734375;
	private static final boolean kBackRightSteerInvert = true;
	private static final double kBackRightXPosInches = -10.625;
	private static final double kBackRightYPosInches = -10.625;

	public static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
					kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide)
			.withSteerMotorInverted(kFrontLeftSteerInvert);
	public static final LegacySwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
					kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide)
			.withSteerMotorInverted(kFrontRightSteerInvert);
	public static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
					kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide)
			.withSteerMotorInverted(kBackLeftSteerInvert);
	public static final LegacySwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
					kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide)
			.withSteerMotorInverted(kBackRightSteerInvert);
}
//package frc2025.subsystems.drive;
//
//import com.ctre.phoenix6.configs.Pigeon2Configuration;
//import com.ctre.phoenix6.configs.Slot0Configs;
//import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
//import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
//import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
//import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
//import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstantsFactory;
//
//import com.pathplanner.lib.config.ModuleConfig;
//import com.pathplanner.lib.config.RobotConfig;
//import edu.wpi.first.math.system.plant.DCMotor;
//import edu.wpi.first.math.util.Units;
//
//public class DriveConstants {
//	// Both sets of gains need to be tuned to your individual robot.
//
//	// The steer motor uses any SwerveModule.SteerRequestType control request with the
//	// output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
//	// VALUES FROM SYSID
//	private static final Slot0Configs steerGains = new Slot0Configs()
//			.withKP(100).withKI(0).withKD(0)
//			.withKS(0).withKV(0.2).withKA(0);
//	// When using closed-loop control, the drive motor uses the control
//	// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
//	private static final Slot0Configs driveGains = new Slot0Configs()
//			.withKP(0.11).withKI(0).withKD(0)
//			.withKS(0.21065).withKV(0.12188).withKA(0.01234);
//
//	// The closed-loop output type to use for the steer motors;
//	// This affects the PID/FF gains for the steer motors
//	private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
//	// The closed-loop output type to use for the drive motors;
//	// This affects the PID/FF gains for the drive motors
//	private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
//
//	// The stator current at which the wheels start to slip;
//	// This needs to be tuned to your individual robot
//	private static final double kSlipCurrentA = 120;
//
//	// Theoretical free speed (m/s) at 12v applied output;
//	// This needs to be tuned to your individual robot
//	public static final double kSpeedAt12VoltsMps = 5.44;
//
//	// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
//	// This may need to be tuned to your individual robot
//	private static final double kCoupleRatio = 3.125;
//
//	private static final double kDriveGearRatio = 5.902777777777778;
//	private static final double kSteerGearRatio = 18.75;
//	//wheel radius (meters) = gyro delta (radians) * drive base radius (meters) / wheel position delta (radians)
//	//private static final double kWheelRadiusInches = 1.989436788649 * 0.93 * 1.04687;
//	private static final double kWheelRadiusInches = 1.957987;
//
//	private static final boolean kSteerMotorReversed = true;
//	private static final boolean kInvertLeftSide = false;
//	private static final boolean kInvertRightSide = true;
//
//	private static final String kCANbusName = "Awesome Sauce";
//	private static final int kPigeonId = 22;
//
//	// These are only used for simulation
//	private static final double kSteerInertia = 0.00001;
//	private static final double kDriveInertia = 0.001;
//	// Simulated voltage necessary to overcome friction
//	private static final double kSteerFrictionVoltage = 0.25;
//	private static final double kDriveFrictionVoltage = 0.25;
//
//	public static final LegacySwerveDrivetrainConstants DrivetrainConstants = new LegacySwerveDrivetrainConstants()
//			.withPigeon2Id(kPigeonId)
//			.withCANbusName(kCANbusName);
//
//	private static final LegacySwerveModuleConstantsFactory ConstantCreator = new LegacySwerveModuleConstantsFactory()
//			.withDriveMotorGearRatio(kDriveGearRatio)
//			.withSteerMotorGearRatio(kSteerGearRatio)
//			.withWheelRadius(kWheelRadiusInches)
//			.withSlipCurrent(kSlipCurrentA)
//			.withSteerMotorGains(steerGains)
//			.withDriveMotorGains(driveGains)
//			.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
//			.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
//			.withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
//			.withSteerInertia(kSteerInertia)
//			.withDriveInertia(kDriveInertia)
//			.withSteerFrictionVoltage(kSteerFrictionVoltage)
//			.withDriveFrictionVoltage(kDriveFrictionVoltage)
//			.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
//			.withCouplingGearRatio(kCoupleRatio)
//			.withSteerMotorInverted(kSteerMotorReversed);
//
//	public static final double kMaxAngularVelocity = 3.14 * 2;
//
//
//	// Front Left
////	private static final int kFrontLeftDriveMotorId = 1;
////	private static final int kFrontLeftSteerMotorId = 2;
////	private static final int kFrontLeftEncoderId = 5;
//	private static final int kFrontLeftDriveMotorId = 0;
//	private static final int kFrontLeftSteerMotorId = 0;
//	private static final int kFrontLeftEncoderId = 0;
//
//	//private static final double kFrontLeftEncoderOffset = -0.367431640625;
//	private static final double kFrontLeftEncoderOffset = 0.337890625;
//	private static final boolean kFrontLeftSteerInvert = true;
//
//	private static final double kFrontLeftXPosInches = 10.625;
//	private static final double kFrontLeftYPosInches = 10.625;
//
//	// Front Right
//	private static final int kFrontRightDriveMotorId = 0;
//	private static final int kFrontRightSteerMotorId = 0;
//	private static final int kFrontRightEncoderId = 0;
//	private static final double kFrontRightEncoderOffset = 0.37060546875;
//	private static final boolean kFrontRightSteerInvert = true;
//
//	private static final double kFrontRightXPosInches = 10.625;
//	private static final double kFrontRightYPosInches = -10.625;
//
//	// Back Left
//	private static final int kBackLeftDriveMotorId = 0;
//	private static final int kBackLeftSteerMotorId = 0;
//	private static final int kBackLeftEncoderId = 0;
//	private static final double kBackLeftEncoderOffset = 0.166015625;
//	private static final boolean kBackLeftSteerInvert = true;
//
//	private static final double kBackLeftXPosInches = -10.625;
//	private static final double kBackLeftYPosInches = 10.625;
//
//	// Back Right
//	private static final int kBackRightDriveMotorId = 0;
//	private static final int kBackRightSteerMotorId = 0;
//	private static final int kBackRightEncoderId = 0;
//	private static final double kBackRightEncoderOffset = 0.013427734375;
//	private static final boolean kBackRightSteerInvert = true;
//	private static final double kBackRightXPosInches = -10.625;
//	private static final double kBackRightYPosInches = -10.625;
//
//	public static final LegacySwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
//					kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide)
//			.withSteerMotorInverted(kFrontLeftSteerInvert);
//	public static final LegacySwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
//					kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide)
//			.withSteerMotorInverted(kFrontRightSteerInvert);
//	public static final LegacySwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
//					kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide)
//			.withSteerMotorInverted(kBackLeftSteerInvert);
//	public static final LegacySwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
//					kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide)
//			.withSteerMotorInverted(kBackRightSteerInvert);
//}