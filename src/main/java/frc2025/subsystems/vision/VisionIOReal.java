package frc2025.subsystems.vision;

import frc2025.RobotState;
import frc2025.config.SubsystemConstants;

import java.util.List;

public class VisionIOReal implements VisionIO {
    List<LimelightFiducialCamera> cameras;

    /*

   TURRET LIMELIGHT CONSTANTS

     */

    public VisionIOReal() {
        this.cameras = List.of(
                new LimelightFiducialCamera("limelight-front", false, SubsystemConstants.robotToFrontCameraForward, SubsystemConstants.robotToFrontCameraRight, SubsystemConstants.robotToFrontCameraUp,
                        SubsystemConstants.robotToFrontCameraYaw, SubsystemConstants.robotToFrontCameraPitch, SubsystemConstants.robotToFrontCameraRoll),
                new LimelightFiducialCamera("limelight-turret", true, 0.0, 0.0, SubsystemConstants.robotToTurretCameraUp,
                        0.0, SubsystemConstants.robotToTurretCameraPitch, SubsystemConstants.robotToTurretCameraRoll)
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
