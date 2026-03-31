package frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

import frc2025.RobotState;

import java.util.List;

public interface VisionIO {
    public class VisionIOInputs {
        public List<RobotState.TimestampedVisionUpdate> mt1Updates;
        public List<RobotState.TimestampedVisionUpdate> mt2Updates;
    }

    public void updateInputs(VisionIOInputs inputs);
    public void resetOdometry();

    public void toggleClimbTags(boolean climbing);
    //public void writeGyros(double yaw);
}
