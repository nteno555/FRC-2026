package frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon.ProtobufPhotonPipelineResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AprilTagFiducialCamera {
    private final String name;
    private final double robotToCameraUp;
    private final double robotToCameraForward;
    private final double robotToCameraRight;
    private final double robotToCameraYaw;
    private final double robotToCameraPitch;
    private final double robotToCameraRoll;

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private ArrayList<Integer> allowedTagIDs = new ArrayList<>();

    private PhotonPoseEstimator multiTagEstimator;

    private PhotonCamera camera;

    private final Transform3d robotToCam;

    private boolean hasSeenFirstPose = false;

    public AprilTagFiducialCamera(String name, double robotToCameraForward,
                                   double robotToCameraRight, double robotToCameraUp,
                                   double robotToCameraYaw, double robotToCameraPitch, double robotToCameraRoll) {

        camera = new PhotonCamera(name);

        this.name = name;
        this.robotToCameraForward = robotToCameraForward;
        this.robotToCameraRight = robotToCameraRight;
        this.robotToCameraUp = robotToCameraUp;
        this.robotToCameraYaw = robotToCameraYaw;
        this.robotToCameraPitch = robotToCameraPitch;
        this.robotToCameraRoll = robotToCameraRoll;

        this.robotToCam = new Transform3d(
                new Translation3d(this.robotToCameraForward, this.robotToCameraRight, this.robotToCameraUp),
                new Rotation3d(this.robotToCameraRoll, this.robotToCameraPitch, this.robotToCameraYaw));

        multiTagEstimator = new PhotonPoseEstimator(kTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    public void resetOdometry() {
        return;
    }

    public void writeGyro() {
        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        omega = Units.radiansToDegrees(omega);
    }

    private Boolean lastAllianceState = null;
    private boolean isClimbing = false;

    private void updateFilters() {
        boolean isRed = lastAllianceState != null ? lastAllianceState : Alliance.get();
        if (isClimbing) {
            if (isRed) {
                this.allowedTagIDs = new ArrayList<>(List.of(15, 16));
            } else {
                this.allowedTagIDs = new ArrayList<>(List.of(31, 32));
            }
        } else {
            if (isRed) {
                this.allowedTagIDs = new ArrayList<>(List.of(2, 3, 4, 5, 8, 9, 10, 11, 25, 26));
            } else {
                this.allowedTagIDs = new ArrayList<>(List.of(9, 10, 18, 19, 20, 21, 24, 25, 26, 27));
            }
        }
    }

    public void updateSide() {
        boolean currentAlliance = Alliance.get();
        if (lastAllianceState == null || lastAllianceState != currentAlliance) {
            lastAllianceState = currentAlliance;
            updateFilters();
        }
    }

    public void toggleClimbTags(boolean climbing) {
        if (this.isClimbing != climbing) {
            this.isClimbing = climbing;
            updateFilters();
        }
    }

    public boolean tagAllowed(int id) {
        for (int allowed : this.allowedTagIDs) {
            if (allowed == id) {
                return true;
            }
        }
        return false;
    }

    private void addVisionUpdateMultiTag(PhotonPipelineResult result) {
        String prefix = "Vision/" + name + "_mt1/";

        double maxMeanDistance = 3.5;
        double maxMeanAmbiguity = 0.2;
        double maxYawRate = Math.PI * 1.67;

        if (!result.hasTargets()) return;

        Optional<EstimatedRobotPose> optionalPose = multiTagEstimator.estimateCoprocMultiTagPose(result);
        boolean usingFallback = false;
        if (optionalPose.isEmpty()) {
            optionalPose = multiTagEstimator.estimateLowestAmbiguityPose(result);
            usingFallback = true;
        }

        Logger.recordOutput(prefix + "isPresent", optionalPose.isPresent());
        Logger.recordOutput(prefix + "usingFallback", usingFallback);
        if (optionalPose.isPresent()) {
            Logger.recordOutput(prefix + "optionalPose", optionalPose.get().estimatedPose.toPose2d());
        }

        if (optionalPose.isEmpty()) return;

        int allowedTagsUsed = (int) optionalPose.get().targetsUsed.stream()
                .filter(t -> tagAllowed(t.getFiducialId())).count();
        Logger.recordOutput(prefix + "allowedTagsUsed", allowedTagsUsed);
        Logger.recordOutput(prefix + "usingFallbackPreCheck", usingFallback);
        Logger.recordOutput(prefix + "allowedTagIDs", allowedTagIDs.stream().mapToInt(Integer::intValue).toArray());
        Logger.recordOutput(prefix + "visibleFiducialIDs", optionalPose.get().targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray());

        if (!usingFallback && allowedTagsUsed < 2) return;
        if (usingFallback && allowedTagsUsed < 1) return;

        EstimatedRobotPose estimate = optionalPose.get();
        Pose2d pose = estimate.estimatedPose.toPose2d();

        List<PhotonTrackedTarget> usedTargets = estimate.targetsUsed.stream()
                .filter(t -> tagAllowed(t.getFiducialId())).toList();

        boolean reject = false;
        double meanDistance = 0;
        double meanAmbiguity = 0;
        double leastDistance = Double.MAX_VALUE;
        boolean meanDistanceReject = false;
        boolean poseAmbiguityReject = false;
        boolean meanAmbiguityReject = false;
        boolean rotationReject = false;
        boolean maxYawRateReject = false;

        for (var t : usedTargets) {
            double dist = t.getBestCameraToTarget().getTranslation().getNorm();
            meanDistance += dist;
            leastDistance = Math.min(leastDistance, dist);
            meanAmbiguity += t.getPoseAmbiguity();

            if (usingFallback) {
                meanAmbiguity += t.getPoseAmbiguity();
                if (t.getPoseAmbiguity() > 0.9) {
                    reject = true;
                    poseAmbiguityReject = true;
                }
            }
        }
        meanDistance /= usedTargets.size();
        meanAmbiguity /= usedTargets.size();

        if (meanDistance > maxMeanDistance) {
            reject = true;
            meanDistanceReject = true;
        }
        if (meanAmbiguity > maxMeanAmbiguity) {
            reject = true;
            meanAmbiguityReject = true;
        }

        // if (hasSeenFirstPose && Math.abs(pose.getRotation()
        //         .minus(RobotState.getInstance().getVisionPose().getRotation())
        //         .getRadians()) > 0.5) {
        //     reject = true;
        //     rotationReject = true;
        // }

        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        if (Math.abs(omega) > maxYawRate) {
            reject = true;
            maxYawRateReject = true;
        }

        Logger.recordOutput(prefix + "tagCount", allowedTagsUsed);
        Logger.recordOutput(prefix + "rejected", reject);
        Logger.recordOutput(prefix + "rawPose", pose);
        Logger.recordOutput(prefix + "meanDistance", meanDistance);
        Logger.recordOutput(prefix + "meanAmbiguity", meanAmbiguity);
        Logger.recordOutput(prefix + "rotationDelta", Math.abs(pose.getRotation()
                .minus(RobotState.getInstance().getVisionPose().getRotation())
                .getRadians()));
        Logger.recordOutput(prefix + "omega", RobotState.getInstance()
                .getFieldRelativeSpeeds().omegaRadiansPerSecond);
        Logger.recordOutput(prefix + "visionPoseRotation",
                RobotState.getInstance().getVisionPose().getRotation().getDegrees());
        Logger.recordOutput(prefix + "estimatedRotation",
                pose.getRotation().getDegrees());
        Logger.recordOutput(prefix + "poseAmbiguityReject", poseAmbiguityReject);
        Logger.recordOutput(prefix + "meanDistanceReject", meanDistanceReject);
        Logger.recordOutput(prefix + "meanAmbiguityReject", meanAmbiguityReject);
        Logger.recordOutput(prefix + "rotationReject", rotationReject);
        Logger.recordOutput(prefix + "maxYawRateReject", maxYawRateReject);

        if (!reject) {
            Vector<N3> distStdDevs;
            if (!usingFallback) {
                distStdDevs = VecBuilder.fill(
                        Math.pow(meanDistance, 2) / 2.0,
                        Math.pow(meanDistance, 2) / 2.0,
                        0.0000001);
            } else {
                distStdDevs = VecBuilder.fill(
                        Math.pow(leastDistance, 2) / 2.0,
                        Math.pow(leastDistance, 2) / 2.0,
                        999999999);
                
            }
            RobotState.getInstance().addVisionData(List.of(
                    new RobotState.TimestampedVisionUpdate(
                            estimate.timestampSeconds, pose, distStdDevs)));
            hasSeenFirstPose = true;
        }
    }

    public void updatePose() {
        updateSide();
        for (var result : camera.getAllUnreadResults()) {
            addVisionUpdateMultiTag(result);
        }
    }
}