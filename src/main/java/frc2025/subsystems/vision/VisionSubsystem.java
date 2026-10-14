package frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.Robot;
import frc2025.RobotState;
import frc2025.config.FieldConstants;

import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    VisionIO io;
    VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

    public VisionSubsystem(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Vision/visionPose", RobotState.getInstance().getVisionPose());
        Logger.recordOutput("Vision/distanceToHub", RobotState.getInstance().getDistanceToHub());
        Logger.recordOutput("Vision/shootingWhileMovingVirtualPose", RobotState.getInstance().getShootingWhileMovingVirtualPose());
        Logger.recordOutput("Vision/hubPosition", FieldConstants.blueNetPos);
//        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");
//        Logger.recordOutput("Vision/testtesttest", poseEstimate.tagCount);

    }

    public Command resetOdometryCommand() {
        return this.runOnce(() -> {
            resetOdometry();
        });
    }

    public void resetOdometry() {
        io.resetOdometry();
    }
}