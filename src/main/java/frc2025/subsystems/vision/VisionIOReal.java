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
                        SubsystemConstants.robotToTurretCameraForward,
                        SubsystemConstants.robotToTurretCameraRight,
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

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        for(var cam: cameras) {
            cam.writeGyro();
            cam.updatePose();
        }
    }

    @Override
    public void resetOdometry() {
        for (var cam: cameras) {
            cam.resetOdometry();
        }
    }
}
