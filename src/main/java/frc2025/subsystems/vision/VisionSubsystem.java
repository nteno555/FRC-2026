package frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.Robot;
import frc2025.RobotState;
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
        Logger.recordOutput("Vision/shooterVisionPose", RobotState.getInstance().getVisionShooterPose());
        //Logger.recordOutput("Vision/shooterVisionPose", RobotState.getInstance().getVisionShooterPose());
        Logger.recordOutput("Vision/visionPoseBuiltin", RobotState.getInstance().getVisionPoseBuiltin());

        Pose2d pose = RobotState.getInstance().getVisionShooterPose();;
        Pose2d target = RobotState.getInstance().getTargetPose();
        double distance = Math.hypot(pose.getX() - target.getX(), pose.getY() - target.getY());
        Logger.recordOutput("Vision/targetDistance", distance);

    }
}
