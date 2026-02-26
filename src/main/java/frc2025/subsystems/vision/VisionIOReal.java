package frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOReal implements VisionIO {
    List<LimelightFiducialCamera> cameras;
    PhotonCamera colorCamera;
    PhotonVisionFiducialCamera photonCamera;

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
                        "limelight-left",
                        false,
                        SubsystemConstants.robotToLeftCameraForward,
                        SubsystemConstants.robotToLeftCameraRight,
                        SubsystemConstants.robotToLeftCameraUp,
                        SubsystemConstants.robotToLeftCameraYaw,
                        SubsystemConstants.robotToLeftCameraPitch,
                        SubsystemConstants.robotToLeftCameraRoll
                )
        );

        colorCamera = new PhotonCamera("colorCamera");
        
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        for(var cam: cameras) {
            cam.writeGyro();
            cam.updatePose();
        }
        photonCamera.updatePose();
        RobotState.getInstance().setBallTargets(colorCamera.getLatestResult().getTargets());
    }

    @Override
    public void resetOdometry() {
        for (var cam: cameras) {
            cam.resetOdometry();
        }
    }
}
