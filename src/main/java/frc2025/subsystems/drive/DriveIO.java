package frc2025.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc2025.subsystems.vision.VisionPoseEstimator;
import org.littletonrobotics.junction.AutoLog;

import java.util.List;

public interface DriveIO {
    @AutoLog
    public class DriveIOInputs {
        public ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        public Pose2d swervePoseMeters = new Pose2d(); // odometry only
        public double odometryTimestamp;
        public Twist2d odometryTwist;
    }

    public void updateInputs(DriveIOInputs inputs);
    public void resetOdometry(Pose2d pose);
    public void setControl(LegacySwerveRequest request);
    public void addVisionMeasurement(List<VisionPoseEstimator.TimestampedVisionUpdate> update);
    public Translation2d[] getModulePositions();
    public LegacySwerveModule[] getModules();
}
