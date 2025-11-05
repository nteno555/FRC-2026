package frc2025.subsystems.drive;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.mechanisms.swerve.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionIO;
import frc2025.subsystems.vision.VisionPoseEstimator;

import java.util.List;

public class DriveIOReal extends LegacySwerveDrivetrain implements DriveIO{
    //VisionPoseEstimator visionPoseEstimator;
    private RobotState robotState;
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private Rotation2d lastGyroYaw = null;

    public DriveIOReal(LegacySwerveDrivetrainConstants driveTrainConstants, LegacySwerveModuleConstants ... modules) {
        super(driveTrainConstants, modules);
        //configureDriveCurrentLimits(100, 0.1, modules);
        //configureTurnCurrentLimits(100, 0.1, modules);
        //visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.currentChassisSpeeds = m_kinematics.toChassisSpeeds(m_moduleStates);
        inputs.swervePoseMeters = m_odometry.getEstimatedPosition();

        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			wheelDeltas[i] = new SwerveModulePosition(
					(m_modulePositions[i].distanceMeters - lastModulePositionsMeters[i]),
					m_modulePositions[i].angle);
			lastModulePositionsMeters[i] = m_modulePositions[i].distanceMeters;
		}
		var twist = m_kinematics.toTwist2d(wheelDeltas);
		var gyroYaw = new Rotation2d(m_pigeon2.getYaw().getValue().in(Units.Radians));
		if (lastGyroYaw != null) {
			twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
		}
		lastGyroYaw = gyroYaw;
        //visionPoseEstimator.addDriveData(Timer.getFPGATimestamp(), new Twist2d(twist.dx, twist.dy, twist.dtheta));
        //inputs.visionPoseMeters = visionPoseEstimator.getLatestPose();
        inputs.odometryTwist = twist;
        inputs.odometryTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        //visionPoseEstimator.resetPose(pose);
        m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
        /*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            setOperatorPerspectiveForward(new Rotation2d(Math.PI));
        } else {
            setOperatorPerspectiveForward(new Rotation2d(0));
        }*/
    }

    @Override
    public void setControl(LegacySwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public void addVisionMeasurement(List<VisionPoseEstimator.TimestampedVisionUpdate> updates) {
        //visionPoseEstimator.addVisionData(updates);
    }

    @Override
    public Translation2d[] getModulePositions() {
        return m_moduleLocations;
    }

    @Override
    public LegacySwerveModule[] getModules() {
        return Modules;
    }

    private void configureDriveCurrentLimits(double limit, double timeout, LegacySwerveModuleConstants[] moduleConstants) {
		for (int i = 0; super.ModuleCount > i; i++) {
			TalonFXConfigurator config = super.Modules[i].getDriveMotor().getConfigurator();

			TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
			configTorque.PeakForwardTorqueCurrent =  moduleConstants[i].SlipCurrent;
			configTorque.PeakReverseTorqueCurrent =  moduleConstants[i].SlipCurrent;
			configTorque.TorqueNeutralDeadband = 1;

			CurrentLimitsConfigs limiterConfigs = new CurrentLimitsConfigs();
			limiterConfigs.SupplyCurrentLimitEnable = true;
			limiterConfigs.SupplyCurrentLimit = limit;
			limiterConfigs.StatorCurrentLimit =  moduleConstants[i].SlipCurrent; // Number from team 33
			limiterConfigs.StatorCurrentLimitEnable = true;

			config.apply(limiterConfigs);
			config.apply(configTorque);
		}

	}

	private void configureTurnCurrentLimits(double limit, double timeout, LegacySwerveModuleConstants[] moduleConstants) {
		for (int i = 0; i < super.ModuleCount; i++) {
			TalonFXConfigurator config = super.Modules[i].getSteerMotor().getConfigurator();

			TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
			configTorque.PeakForwardTorqueCurrent = moduleConstants[i].SlipCurrent;
			configTorque.PeakReverseTorqueCurrent = -moduleConstants[i].SlipCurrent;
			configTorque.TorqueNeutralDeadband = 1;

			CurrentLimitsConfigs limiterConfigs = new CurrentLimitsConfigs();
			limiterConfigs.SupplyCurrentLimitEnable = true;
			limiterConfigs.SupplyCurrentLimit = limit;
			limiterConfigs.StatorCurrentLimit = moduleConstants[i].SlipCurrent;
			limiterConfigs.StatorCurrentLimitEnable = true;

			config.apply(limiterConfigs);
			config.apply(configTorque);
		}
	}
}
