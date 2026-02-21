package frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;

import frc2025.RobotState;

import java.util.List;
import java.util.Optional;

public class PhotonVisionFiducialCamera {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private static final AprilTagFieldLayout kTagLayoutWelded = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final AprilTagFieldLayout kTagLayoutAndymark = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
                

    public PhotonVisionFiducialCamera(String name, boolean fieldType, double forward, double side, double up, double yaw, double pitch, double roll) {
        this.camera = new PhotonCamera(name);
        Transform3d kRobotToCam = new Transform3d(new Translation3d(forward, side, up), new Rotation3d(roll, pitch, yaw));
        if (fieldType) {
            this.poseEstimator = new PhotonPoseEstimator(kTagLayoutWelded, kRobotToCam);
        } else {
            this.poseEstimator = new PhotonPoseEstimator(kTagLayoutAndymark, kRobotToCam);
        }

        this.poseEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        this.poseEstimator.setRobotToCameraTransform(kRobotToCam);
    }

    public void updatePose() {

        Optional<EstimatedRobotPose> estimateOpt = poseEstimator.estimateCoprocMultiTagPose(camera.getLatestResult());

        if (estimateOpt.isEmpty()) {
            estimateOpt = poseEstimator.estimateLowestAmbiguityPose(camera.getLatestResult());
        }

        if (estimateOpt.isEmpty()) {
            return;
        }

        EstimatedRobotPose estimate = estimateOpt.get();
        Pose2d visionPose2d = estimate.estimatedPose.toPose2d();

        double maxYawRate = Math.PI * 2;
        double fieldYawRate = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;

        if (Math.abs(fieldYawRate) > maxYawRate) {
            return;
        }

        var results = this.camera.getLatestResult();
        double tagCount = results.getTargets().size();
        if (tagCount == 0) {return;}
        double totalDistance = 0.0;
        for (var target : results.getTargets()) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        double avgDistance = totalDistance / tagCount;


        double xyStdDev = 0.03 * avgDistance;


        xyStdDev /= Math.sqrt(tagCount);


        double thetaStdDev = 0.08 * avgDistance;

        Vector<N3> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev,thetaStdDev);

        VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
                estimate.timestampSeconds, visionPose2d, stdDevs
        );

        RobotState.getInstance().addVisionData(List.of(update));
    }
}