package frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        Logger.recordOutput("Vision/swervePose", RobotState.getInstance().getSwervePose());
    }
}
