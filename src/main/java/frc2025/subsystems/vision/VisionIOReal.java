package frc2025.subsystems.vision;

import frc2025.RobotState;
import frc2025.config.ShooterTreeMap;

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
    PhotonCamera photonCamera;
    /*

   TURRET LIMELIGHT CONSTANTS

     */

    public VisionIOReal() {
        this.cameras = List.of(
                //new LimelightFiducialCamera("limelight-xright"),
                new LimelightFiducialCamera("limelight-front", false),
                new LimelightFiducialCamera("limelight-turret", true)
        );
        photonCamera = new PhotonCamera("colorCamera");
        
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        for(var cam: cameras) {
            cam.writeGyro();
            cam.updatePose();
        }
        RobotState.getInstance().setBallTarget(photonCamera.getLatestResult().getBestTarget());
    }

}
