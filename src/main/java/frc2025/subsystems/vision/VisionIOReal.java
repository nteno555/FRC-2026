package frc2025.subsystems.vision;

import frc2025.RobotState;

import java.util.List;

public class VisionIOReal implements VisionIO {
    List<LimelightFiducialCamera> cameras;

    public VisionIOReal() {
        this.cameras = List.of(
                //new LimelightFiducialCamera("limelight-xright"),
                new LimelightFiducialCamera("limelight-xleft")
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        for(var cam: cameras) {
            cam.writeGyro();
            cam.updatePose();
        }
    }
}
