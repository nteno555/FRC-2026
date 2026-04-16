package frc2025.subsystems.drive;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionIO;
import org.littletonrobotics.junction.Logger;


import java.util.List;

public class DriveIOReal extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO{
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private Rotation2d lastGyroYaw = null;
    private Pose2d lastOdometryPose = null;
    private double lastOdometryPoseTimestamp = 0;

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

    private boolean slipping = false;

    private int slipCounter = 0;

    private double lastTime = Timer.getFPGATimestamp();

    private final Matrix<N3, N1> normalStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), 
        new double[]{0.1, 0.1, 0.1});

    private final Matrix<N3, N1> slippingStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), 
        new double[]{99999, 99999, 0.1});

    public DriveIOReal(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);
    }


    @Override
    public void updateInputs(DriveIOInputs inputs) {
        isSlipping();

        var state = getState();
        inputs.swervePoseMeters = state.Pose;
        lastOdometryPose = inputs.swervePoseMeters;
        inputs.currentChassisSpeeds = getKinematics().toChassisSpeeds(state.ModuleStates);

//        Matrix<N3, N1> regStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {0.1, 0.1, 0.1});
//        Matrix<N3, N1> slippingStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {99999, 99999, 0.1});
//
//        if (!isSlipping()) {
//            this.setStateStdDevs(regStdDevs);
//
//            inputs.swervePoseMeters = state.Pose;
//            lastOdometryPose = inputs.swervePoseMeters;
//            inputs.currentChassisSpeeds = getKinematics().toChassisSpeeds(state.ModuleStates);
//            lastGoodChassisSpeeds = inputs.currentChassisSpeeds;
//        }
//        else {
//            this.setStateStdDevs(slippingStdDevs);
//
//            inputs.swervePoseMeters = state.Pose;
//            lastOdometryPose = inputs.swervePoseMeters;
//            inputs.currentChassisSpeeds = getKinematics().toChassisSpeeds(state.ModuleStates);
//            lastGoodChassisSpeeds = inputs.currentChassisSpeeds;
//
//        }

        SwerveModule[] modules = getModules();
        double driveCurrent = 0;
        for (int i = 0; i < modules.length; i++) {
            Logger.recordOutput("Current/Drive Motor "+(i+1), modules[i].getDriveMotor().getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Current/Steer Motor "+(i+1), modules[i].getSteerMotor().getStatorCurrent().getValueAsDouble());
            driveCurrent += modules[i].getDriveMotor().getStatorCurrent().getValueAsDouble();
            driveCurrent += modules[i].getSteerMotor().getStatorCurrent().getValueAsDouble();
        }
        inputs.totalDriveCurrent = driveCurrent;

//        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
//        for (int i = 0; i < 4; i++) {
//            wheelDeltas[i] = new SwerveModulePosition(
//                    (state.ModulePositions[i].distanceMeters - lastModulePositionsMeters[i]),
//                    state.ModulePositions[i].angle);
//            lastModulePositionsMeters[i] = state.ModulePositions[i].distanceMeters;
//        }
//        var twist = getKinematics().toTwist2d(wheelDeltas);
//        Rotation2d gyroYaw = Rotation2d.fromRadians(getPigeon2().getYaw().getValue().in(Units.Radians));
//        if (lastGyroYaw != null) {
//            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
//        }
//        lastGyroYaw = gyroYaw;
//        inputs.odometryTwist = twist;
//        inputs.odometryTimestamp = Timer.getFPGATimestamp();
//
//        double xAccel = getPigeon2().getAccelerationX().getValue().in(Units.MetersPerSecondPerSecond);
//        double yAccel = getPigeon2().getAccelerationY().getValue().in(Units.MetersPerSecondPerSecond);
//        inputs.pigeonAcceleration = Math.hypot(xAccel, yAccel);

//        Logger.recordOutput("Drive/pigeonRotation", getPigeon2().getRotation2d());
//        Logger.recordOutput("Drive/pigeonConnected", getPigeon2().isConnected());
//        Logger.recordOutput("Drive/SwerveModuleStates", getState().ModuleStates);
//        Logger.recordOutput("Drive/SwerveModuleStatesWanted", getState().ModuleTargets);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(List<RobotState.TimestampedVisionUpdate> updates) {
        double avgTimestamp = 0;
        for (var update: updates) {
//            Logger.recordOutput("Drive/addVisionMeasurementHeartbeat2", Math.random());
//            Logger.recordOutput("Drive/visionUpdatePose", update.pose());
//            Logger.recordOutput("Drive/visionUpdateTimestamp", update.timestamp());
//            Logger.recordOutput("Drive/currentTimestamp", Utils.getCurrentTimeSeconds());

            double timestamp = Utils.fpgaToCurrentTime(update.timestamp());
            avgTimestamp += timestamp;
            addVisionMeasurement(update.pose(), timestamp, update.stdDevs());
        }
        avgTimestamp /= updates.size();

        Logger.recordOutput("Drive/visionLatency", Utils.getCurrentTimeSeconds() - avgTimestamp);
    }

    @Override
    public Translation2d[] getModulePositions() {
        return getModuleLocations();
    }


    private boolean isSlipping() {
        var speeds = getState().Speeds;
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;

        if (dt <= 0) {
            return slipping;
        }

        double robotSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double wheelAccel = Math.hypot((speeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) / dt, (speeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) / dt);
        
        double imuAccel = Math.hypot(getPigeon2().getAccelerationX().getValue().in(Units.MetersPerSecondPerSecond), getPigeon2().getAccelerationY().getValue().in(Units.MetersPerSecondPerSecond));

        this.lastSpeeds = speeds;
        this.lastTime = now;
        
        boolean slipCondition = (robotSpeed > 0.2) && (wheelAccel > 4.0) && (imuAccel < (wheelAccel * 0.4)); // tune
        Logger.recordOutput("Drive/wheelAccel", wheelAccel);
        Logger.recordOutput("Drive/imuAccel", imuAccel);
        Logger.recordOutput("Drive/robotSpeed", robotSpeed);
        Logger.recordOutput("Drive/slipCondition", slipCondition);

        if (slipCondition) {
            this.slipCounter++;
        } else {
            if (slipCounter > 0) {slipCounter--;}
        }
        
        this.slipping = this.slipCounter >= 3;
        
        if (this.slipping) {
            this.setStateStdDevs(this.slippingStdDevs);
        } else {
            this.setStateStdDevs(this.normalStdDevs);
        }
        
        Logger.recordOutput("Drive/Slipping", this.slipping);
        return this.slipping;
    }


    /*private void configureDriveCurrentLimits(double limit, double timeout, SwerveModuleConstants[] moduleConstants) {
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
    }*/
}
