package frc2025.subsystems.drive;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionIO;
import frc2025.subsystems.vision.VisionPoseEstimator;


import java.util.List;

public class DriveIOReal extends SwerveDrivetrain implements DriveIO{
    private final Translation2d[] moduleLocations;
    private final Pigeon2 pigeon2;
    //    private final SwerveModule[] modules;
    //VisionPoseEstimator visionPoseEstimator;
    private RobotState robotState;
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private Rotation2d lastGyroYaw = null;

    public DriveIOReal(SwerveDrivetrainConstants driveTrainConstants, Translation2d[] moduleLocations, SwerveModuleConstants... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);
        this.moduleLocations = moduleLocations;
        this.robotState = RobotState.getInstance();
        this.pigeon2 = new Pigeon2(6741); // supposed to be CAN ID
    }


    @Override
    public void updateInputs(DriveIOInputs inputs) {
        var state = getState();
        inputs.currentChassisSpeeds = getKinematics().toChassisSpeeds(state.ModuleStates);
        inputs.swervePoseMeters = robotState.getVisionPose();

        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (state.ModulePositions[i].distanceMeters - lastModulePositionsMeters[i]),
                    state.ModulePositions[i].angle);
            lastModulePositionsMeters[i] = state.ModulePositions[i].distanceMeters;
        }
        var twist = getKinematics().toTwist2d(wheelDeltas);
        Rotation2d gyroYaw = Rotation2d.fromRadians(pigeon2.getYaw().getValue().in(Units.Radians));
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
        robotState.setSwervePose(pose);
        lastGyroYaw = pigeon2.getRotation2d();
        /*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            setOperatorPerspectiveForward(new Rotation2d(Math.PI));
        } else {
            setOperatorPerspectiveForward(new Rotation2d(0));
        }*/
    }

    @Override
    public void setControl(SwerveRequest request) {
        this.setControl(request);
    }

    @Override
    public void addVisionMeasurement(List<VisionPoseEstimator.TimestampedVisionUpdate> updates) {
        //visionPoseEstimator.addVisionData(updates);
    }

    @Override
    public Translation2d[] getModulePositions() {
        return moduleLocations;
    }

    private void configureDriveCurrentLimits(double limit, double timeout, SwerveModuleConstants[] moduleConstants) {
        SwerveModule[] modules = getModules();
        for (int i = 0; modules.length > i; i++) {
            TalonFX driveMotor = (TalonFX) modules[i].getDriveMotor();
            TorqueCurrentConfigs torqueConfig = new TorqueCurrentConfigs();
            torqueConfig.PeakForwardTorqueCurrent = moduleConstants[i].SlipCurrent;
            torqueConfig.PeakReverseTorqueCurrent = moduleConstants[i].SlipCurrent;
            torqueConfig.TorqueNeutralDeadband = 1;
            driveMotor.getConfigurator().apply(torqueConfig);

            CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
            limits.SupplyCurrentLimitEnable = true;
            limits.SupplyCurrentLimit = limit;
            limits.StatorCurrentLimitEnable = true;
            limits.StatorCurrentLimit = moduleConstants[i].SlipCurrent;
            driveMotor.getConfigurator().apply(limits);
        }

    }

    private void configureTurnCurrentLimits(double limit, double timeout, SwerveModuleConstants[] moduleConstants) {
        SwerveModule[] modules = getModules();
        for (int i = 0; i < modules.length; i++) {
            TalonFXConfigurator config = modules[i].getSteerMotor().getConfigurator();

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
