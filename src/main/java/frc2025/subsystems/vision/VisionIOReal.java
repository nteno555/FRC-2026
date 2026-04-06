package frc2025.subsystems.vision;

import frc2025.config.SubsystemConstants;

import java.util.List;

public class VisionIOReal implements VisionIO {
    List<LimelightFiducialCamera> cameras;

    public VisionIOReal() {
        this.cameras = List.of(
                new LimelightFiducialCamera(
                        "limelight-turret",
                        true,
                        0.0,
                        0.0,
                        SubsystemConstants.robotToTurretCameraUp,
                        SubsystemConstants.robotToTurretCameraYaw,
                        SubsystemConstants.robotToTurretCameraPitch,
                        SubsystemConstants.robotToTurretCameraRoll
                ),
                new LimelightFiducialCamera(
                        "limelight-back",
                        false,
                        SubsystemConstants.robotToBackCameraForward,
                        SubsystemConstants.robotToBackCameraRight,
                        SubsystemConstants.robotToBackCameraUp,
                        SubsystemConstants.robotToBackCameraYaw,
                        SubsystemConstants.robotToBackCameraPitch,
                        SubsystemConstants.robotToBackCameraRoll
                ),
                new LimelightFiducialCamera(
                        "limelight-right",
                        false,
                        SubsystemConstants.robotToRightCameraForward,
                        SubsystemConstants.robotToRightCameraRight,
                        SubsystemConstants.robotToRightCameraUp,
                        SubsystemConstants.robotToRightCameraYaw,
                        SubsystemConstants.robotToRightCameraPitch,
                        SubsystemConstants.robotToRightCameraRoll
                ),
               new LimelightFiducialCamera(
                       "limelight-front",
                       false,
                       SubsystemConstants.robotToFrontCameraForward,
                       SubsystemConstants.robotToFrontCameraRight,
                       SubsystemConstants.robotToFrontCameraUp,
                       SubsystemConstants.robotToFrontCameraYaw,
                       SubsystemConstants.robotToFrontCameraPitch,
                       SubsystemConstants.robotToFrontCameraRoll
               )
        );
    }

    private int cameraIndex = 0;
 
     @Override
     public void updateInputs(VisionIOInputs inputs) {
         if (cameras.isEmpty()) return;
 
         // Update gyro orientation for all cameras (fast NT write, no flush)
         for (var cam : cameras) {
             cam.writeGyro();
             cam.updatePose();
         }
 
         // Process pose for only one camera per frame (round-robin)
//         cameras.get(cameraIndex).updatePose();
//         cameraIndex = (cameraIndex + 1) % cameras.size();
 
         // Flush all orientation/gyro updates to NetworkTables once per loop
         LimelightHelpers.Flush();
     }

    @Override
    public void resetOdometry() {
        for (var cam: cameras) {
            cam.resetOdometry();
        }
    }

    @Override
    public void toggleClimbTags(boolean climbing) {
        for (var cam: cameras) {
            cam.toggleClimbTags(climbing);
        }
    }
}
