package frc2025.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionPoseEstimator;
import org.littletonrobotics.junction.AutoLog;

import java.util.List;
import java.util.Optional;

public interface DriveIO{
    @AutoLog
    public class DriveIOInputs {
        public ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        public Pose2d swervePoseMeters = new Pose2d(); // odometry only
        public double odometryTimestamp;
        public Twist2d odometryTwist;
        public double pigeonAcceleration;
        public double[] driveCurrent = new double[4];
        public double[] driveVoltage = new double[4];
        public double[] turnCurrent = new double[4];
        public double[] turnVoltage = new double[4];
    }

    public void updateInputs(DriveIOInputs inputs);
    public void resetOdometry(Pose2d pose);
    public void setControl(SwerveRequest request);
    public void addVisionMeasurement(List<RobotState.TimestampedVisionUpdate> updates);
    public Translation2d[] getModulePositions();
    public SwerveModule[] getModules();
    public Optional<Pose2d> samplePoseAt(double timestamp);
}
