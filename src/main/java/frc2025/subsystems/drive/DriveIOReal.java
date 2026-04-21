package frc2025.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.Debouncer;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionPoseEstimator;

import org.littletonrobotics.junction.Logger;

import java.util.List;

public class DriveIOReal extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
    private double[] lastModulePositionsMeters = new double[] { 0.0, 0.0, 0.0, 0.0 };
    private Rotation2d lastGyroYaw = null;
    private Pose2d lastOdometryPose = null;
    private double lastOdometryPoseTimestamp = 0;
    private ChassisSpeeds lastGoodChassisSpeeds = null;
    private double lastVisionUpdateTimestamp = 0;
    private final Debouncer slipDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    public DriveIOReal(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        var state = getState();

        Matrix<N3, N1> regStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 0.1, 0.1, 0.1 });
        Matrix<N3, N1> slippingStdDevs = new Matrix<>(Nat.N3(), Nat.N1(),
                new double[] { 9999999999.0, 9999999999.0, 0.1 });

        boolean isSlipping = slipDebouncer.calculate(isSlipping());
        Logger.recordOutput("Drive/slipping", isSlipping);

        if (!isSlipping) {
            this.setStateStdDevs(regStdDevs);
            inputs.currentChassisSpeeds = getKinematics().toChassisSpeeds(state.ModuleStates);
            lastGoodChassisSpeeds = inputs.currentChassisSpeeds;
            lastOdometryPose = state.Pose;
            inputs.swervePoseMeters = state.Pose;
        } else {
            this.setStateStdDevs(slippingStdDevs);
            inputs.currentChassisSpeeds = lastGoodChassisSpeeds;

            if (Timer.getFPGATimestamp() - lastVisionUpdateTimestamp > 0.15) {
                // Pin the robot translation since we have no vision and are slipping
                Pose2d pinningPose = new Pose2d(lastOdometryPose.getTranslation(), state.Pose.getRotation());
                Matrix<N3, N1> pinStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 0.01, 0.01, 99999.0 });
                this.addVisionMeasurement(pinningPose, Utils.getCurrentTimeSeconds(), pinStdDevs);
                inputs.swervePoseMeters = pinningPose;
            } else {
                inputs.swervePoseMeters = state.Pose;
            }
        }

        // SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        // for (int i = 0; i < 4; i++) {
        // wheelDeltas[i] = new SwerveModulePosition(
        // (state.ModulePositions[i].distanceMeters - lastModulePositionsMeters[i]),
        // state.ModulePositions[i].angle);
        // lastModulePositionsMeters[i] = state.ModulePositions[i].distanceMeters;
        // }
        // var twist = getKinematics().toTwist2d(wheelDeltas);
        // Rotation2d gyroYaw =
        // Rotation2d.fromRadians(getPigeon2().getYaw().getValue().in(Units.Radians));
        // if (lastGyroYaw != null) {
        // twist = new Twist2d(twist.dx, twist.dy,
        // gyroYaw.minus(lastGyroYaw).getRadians());
        // }
        // lastGyroYaw = gyroYaw;
        // inputs.odometryTwist = twist;
        // inputs.odometryTimestamp = Timer.getFPGATimestamp();
        //
        // double xAccel =
        // getPigeon2().getAccelerationX().getValue().in(Units.MetersPerSecondPerSecond);
        // double yAccel =
        // getPigeon2().getAccelerationY().getValue().in(Units.MetersPerSecondPerSecond);
        // inputs.pigeonAcceleration = Math.hypot(xAccel, yAccel);

        // Logger.recordOutput("Drive/pigeonRotation", getPigeon2().getRotation2d());
        // Logger.recordOutput("Drive/pigeonConnected", getPigeon2().isConnected());
        // Logger.recordOutput("Drive/SwerveModuleStates", getState().ModuleStates);
        // Logger.recordOutput("Drive/SwerveModuleStatesWanted",
        // getState().ModuleTargets);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(List<RobotState.TimestampedVisionUpdate> updates) {
        double avgTimestamp = 0;
        for (var update : updates) {
            // Logger.recordOutput("Drive/addVisionMeasurementHeartbeat2", Math.random());
            // Logger.recordOutput("Drive/visionUpdatePose", update.pose());
            // Logger.recordOutput("Drive/visionUpdateTimestamp", update.timestamp());
            // Logger.recordOutput("Drive/currentTimestamp", Utils.getCurrentTimeSeconds());

            double timestamp = Utils.fpgaToCurrentTime(update.timestamp());
            avgTimestamp += timestamp;
            addVisionMeasurement(update.pose(), timestamp, update.stdDevs());
        }
        if (!updates.isEmpty()) {
            lastVisionUpdateTimestamp = Timer.getFPGATimestamp();
        }
        avgTimestamp /= updates.size();

        Logger.recordOutput("Drive/visionLatency", Utils.getCurrentTimeSeconds() - avgTimestamp);
    }

    @Override
    public Translation2d[] getModulePositions() {
        return getModuleLocations();
    }

    private boolean isSlipping() {
        var state = getState();
        SwerveModuleState[] moduleStates = state.ModuleStates;
        Translation2d[] moduleLocation = getModuleLocations();
        double omega = getPigeon2().getAngularVelocityZWorld().getValue().in(Units.RadiansPerSecond);

        double maxTransVelocity = 0.0;
        double minTransVelocity = Double.MAX_VALUE;

        for (int i = 0; i < 4; i++) {
            double velocity = moduleStates[i].speedMetersPerSecond;
            double angle = moduleStates[i].angle.getRadians();

            double vx = velocity * Math.cos(angle);
            double vy = velocity * Math.sin(angle);

            double rotationalX = -omega * moduleLocation[i].getY();
            double rotationalY = omega * moduleLocation[i].getX();

            double translationalX = vx - rotationalX;
            double translationalY = vy - rotationalY;

            double transVelMagnitude = Math.hypot(translationalX, translationalY);

            maxTransVelocity = Math.max(maxTransVelocity, transVelMagnitude);
            minTransVelocity = Math.min(minTransVelocity, transVelMagnitude);
        }

        if (maxTransVelocity <= 0.2) {
            Logger.recordOutput("Drive/Slip/maxTransVelocity", maxTransVelocity);
            Logger.recordOutput("Drive/Slip/minTransVelocity", minTransVelocity);
            Logger.recordOutput("Drive/Slip/transVelocityRatio", 1.0);
            Logger.recordOutput("Drive/slipping", false);
            return false;
        }

        double ratio = maxTransVelocity / minTransVelocity;

        Logger.recordOutput("Drive/Slip/maxTransVelocity", maxTransVelocity);
        Logger.recordOutput("Drive/Slip/minTransVelocity", minTransVelocity);
        Logger.recordOutput("Drive/Slip/ratio", ratio);

        boolean slipping = ratio > 10;
        Logger.recordOutput("Drive/slipping", slipping);

        return slipping;
    }
    /*
     * private void configureDriveCurrentLimits(double limit, double timeout,
     * SwerveModuleConstants[] moduleConstants) {
     * SwerveModule[] modules = getModules();
     * for (int i = 0; modules.length > i; i++) {
     * TalonFX driveMotor = (TalonFX) modules[i].getDriveMotor();
     * TorqueCurrentConfigs torqueConfig = new TorqueCurrentConfigs();
     * torqueConfig.PeakForwardTorqueCurrent = moduleConstants[i].SlipCurrent;
     * torqueConfig.PeakReverseTorqueCurrent = moduleConstants[i].SlipCurrent;
     * torqueConfig.TorqueNeutralDeadband = 1;
     * driveMotor.getConfigurator().apply(torqueConfig);
     * 
     * CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
     * limits.SupplyCurrentLimitEnable = true;
     * limits.SupplyCurrentLimit = limit;
     * limits.StatorCurrentLimitEnable = true;
     * limits.StatorCurrentLimit = moduleConstants[i].SlipCurrent;
     * driveMotor.getConfigurator().apply(limits);
     * }
     * 
     * }
     * 
     * private void configureTurnCurrentLimits(double limit, double timeout,
     * SwerveModuleConstants[] moduleConstants) {
     * SwerveModule[] modules = getModules();
     * for (int i = 0; i < modules.length; i++) {
     * TalonFXConfigurator config = modules[i].getSteerMotor().getConfigurator();
     * 
     * TorqueCurrentConfigs configTorque = new TorqueCurrentConfigs();
     * configTorque.PeakForwardTorqueCurrent = moduleConstants[i].SlipCurrent;
     * configTorque.PeakReverseTorqueCurrent = -moduleConstants[i].SlipCurrent;
     * configTorque.TorqueNeutralDeadband = 1;
     * 
     * CurrentLimitsConfigs limiterConfigs = new CurrentLimitsConfigs();
     * limiterConfigs.SupplyCurrentLimitEnable = true;
     * limiterConfigs.SupplyCurrentLimit = limit;
     * limiterConfigs.StatorCurrentLimit = moduleConstants[i].SlipCurrent;
     * limiterConfigs.StatorCurrentLimitEnable = true;
     * 
     * config.apply(limiterConfigs);
     * config.apply(configTorque);
     * }
     * }
     */
}