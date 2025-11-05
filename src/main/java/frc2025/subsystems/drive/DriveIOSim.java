package frc2025.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc2025.subsystems.vision.VisionPoseEstimator;

import java.util.List;

public class DriveIOSim extends DriveIOReal {

    public DriveIOSim(LegacySwerveDrivetrainConstants driveTrainConstants, LegacySwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        updateSimState(0.02, RobotController.getBatteryVoltage());
        super.updateInputs(inputs);
    }
}
