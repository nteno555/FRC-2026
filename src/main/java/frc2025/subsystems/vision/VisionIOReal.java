package frc2025.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;

import java.util.ArrayList;
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
    ArrayList<AprilTagFiducialCamera> cameras;
    PhotonCamera ballCamera;
    // PhotonCamera colorCamera;
    // //PhotonVisionFiducialCamera photonCamera;

    public VisionIOReal() {
        // this.cameras = new LimelightFiducialCamera("limelight", 
        //         0.32385, 
        //         0, 
        //         0.10414, 
        //         0,
        //         49, 
        //         180);
        this.cameras = new ArrayList<>(List.of(
            new AprilTagFiducialCamera(
                "front-right", 
                0, 
                0, 
                0, 
                0,
                0,
                0),
            new AprilTagFiducialCamera(
                "side-right", 
                0, 
                0, 
                0, Math.PI/2, 
                0, 
                Math.PI)
        ));
        
        ballCamera = new PhotonCamera("ballCamera");

        
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        for(var cam: cameras) {
            cam.writeGyro();
            cam.updatePose();
        }
        // cameras.writeGyro();
        // cameras.updatePose();
        //photonCamera.updatePose();
        RobotState.getInstance().setBallTargets(ballCamera.getLatestResult().getTargets());
    }

    @Override
    public void resetOdometry() {
        for (var cam: cameras) {
            cam.resetOdometry();
        }
        // cameras.resetOdometry();
    }
}
