package frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;

public interface VisionIO {
    public class VisionIOInputs {
        public List<VisionPoseEstimator.TimestampedVisionUpdate> mt1Updates;
        public List<VisionPoseEstimator.TimestampedVisionUpdate> mt2Updates;
    }

    public void updateInputs(VisionIOInputs inputs);
    public Pose2d detectObjectPose();
    //public void writeGyros(double yaw);
}
