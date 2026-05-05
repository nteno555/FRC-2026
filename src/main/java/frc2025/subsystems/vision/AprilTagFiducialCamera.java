
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

    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private ArrayList<Integer> allowedTagIDs = new ArrayList<>();

    // private boolean gyroInitialized = false;
    // private double resetOdometryTime = 0.0;
    // private boolean flushNextOrientation = false;

    private PhotonPoseEstimator multiTagEstimator;
    private PhotonPoseEstimator constrainedPnPEstimator;

    private PhotonCamera camera;

    private final Transform3d robotToCam;

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

        this.robotToCam = new Transform3d(new Translation3d(this.robotToCameraForward, this.robotToCameraRight, this.robotToCameraUp), new Rotation3d(this.robotToCameraRoll, this.robotToCameraPitch, this.robotToCameraYaw));
        

        multiTagEstimator = new PhotonPoseEstimator(kTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        constrainedPnPEstimator = new PhotonPoseEstimator(kTagLayout, PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP, robotToCam);
    }

    public void resetOdometry() {
//        resetOdometryTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
//        flushNextOrientation = true;
//        LimelightHelpers.SetIMUMode(name, 0);
        return;
    }

    // public void initializeGyro() {
    //     if (this.turretMounted) {
    //         LimelightHelpers.SetIMUMode(name, 3);
    //         LimelightHelpers.SetIMUAssistAlpha(name, 0.01);
    //         LimelightHelpers.setCameraPose_RobotSpace(name, 0.0, 0.0, this.robotToCameraUp,
    //             this.robotToCameraRoll, this.robotToCameraPitch, 0.0);
    //     } else {
    //         LimelightHelpers.SetIMUMode(name, 4);
    //         LimelightHelpers.SetIMUAssistAlpha(name, 0.01);
    //         LimelightHelpers.setCameraPose_RobotSpace(name, this.robotToCameraForward, this.robotToCameraRight,
    //             this.robotToCameraUp,
    //             this.robotToCameraRoll, this.robotToCameraPitch, this.robotToCameraYaw);
    //     }

    //     this.gyroInitialized = true;

    // }

    public void writeGyro() {
        // String prefix = "Vision/" + name + "_mt2/";

//        if(imuMode != 4) {
//            LimelightHelpers.SetIMUAssistAlpha(name, 0.01);
//            LimelightHelpers.SetIMUMode(name, 4);
//            imuMode = 4;
//        }

//        Logger.recordOutput(prefix + "IMU", imuMode);

       
        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        omega = Units.radiansToDegrees(omega);

        this.constrainedPnPEstimator.addHeadingData(Timer.getFPGATimestamp(), RobotState.getInstance().getVisionPose().getRotation());

//            if (flushNextOrientation) {
//                LimelightHelpers.SetRobotOrientation(name,
//                        RobotState.getInstance().getVisionPose().getRotation().getDegrees(), omega, 0, 0, 0, 0);
//            } else {
//                LimelightHelpers.SetRobotOrientation_NoFlush(name,
//                        RobotState.getInstance().getVisionPose().getRotation().getDegrees(), omega, 0, 0, 0, 0);
//            }
        
        
        // flushNextOrientation = false;
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
                this.allowedTagIDs = new ArrayList<>(List.of( 2, 3, 4, 5, 8, 9, 10, 11, 25, 26 ));
            } else {
                this.allowedTagIDs = new ArrayList<>(List.of( 9, 10, 18, 19, 20, 21, 24, 25, 26, 27 ));
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

//     private Pose2d transformTurretPose(Pose2d pose, double time) {
//         String prefix = "Vision/" + name + "_mt1/";

//         if (SubsystemConstants.hasTurret) {
//             double latency = Utils.getCurrentTimeSeconds() - Utils.fpgaToCurrentTime(time);

//             Logger.recordOutput(prefix + "latency", latency);

//             double compensatedTurretAngle = RobotState.getInstance().getTurretAngle() 
//                     + RobotState.getInstance().getTurretVelocity() * latency;

//             Transform2d robotToShooterReal = new Transform2d(
//                     SubsystemConstants.robotToShooter.getTranslation(),
//                     SubsystemConstants.robotToShooter.getRotation()
//                             .plus(Rotation2d.fromRadians(-compensatedTurretAngle)));

//             Transform2d cameraToRobotTransform = robotToShooterReal.plus(SubsystemConstants.shooterToCamera).inverse();

// //            Logger.recordOutput("Vision/" + name + "_mt2/turretTransform",
// //                    cameraToRobotTransform.getRotation().getDegrees());

//             return pose.transformBy(cameraToRobotTransform);
//         } else {
//             Transform2d cameraToRobotTransform = SubsystemConstants.robotToShooter
//                     .plus(SubsystemConstants.shooterToCamera).inverse();
//             return pose.transformBy(cameraToRobotTransform);
//         }
//     }

    public void addVisionUpdate(PhotonPipelineResult result) {
        String prefix = "Vision/" + name + "/";

        double maxMeanDistance = 2.5;
        double maxMeanAmbiguity = 0.2;
        double maxYawRate = Math.PI * 1.67; // 300 deg/s

        if (!result.hasTargets()) return;

        boolean usingConstrained = false;
        Optional<EstimatedRobotPose> optionalPose = multiTagEstimator.estimateCoprocMultiTagPose(result);

        int allowedTagsUsed = 0;
        if (optionalPose.isPresent()) {
            allowedTagsUsed = (int) optionalPose.get().targetsUsed.stream().filter(t -> tagAllowed(t.getFiducialId())).count();
        }

        if (optionalPose.isEmpty() || allowedTagsUsed < 2) {
            var calibData = camera.getCameraMatrix();
            var distCoeffs = camera.getDistCoeffs();
            if (calibData.isEmpty() || distCoeffs.isEmpty()) return;

            Pose3d seedPose = new Pose3d(RobotState.getInstance().getVisionPose());
            optionalPose = constrainedPnPEstimator.estimateConstrainedSolvepnpPose(
                    result,
                    calibData.get(),
                    distCoeffs.get(),
                    seedPose,
                    false,
                    0.5);

            if (optionalPose.isEmpty()) return;
            usingConstrained = true;
        }

        EstimatedRobotPose estimate = optionalPose.get();
        Pose2d pose = estimate.estimatedPose.toPose2d();

        int tagCount = 0;
        for (PhotonTrackedTarget tag : estimate.targetsUsed) {
            if (tagAllowed(tag.getFiducialId())) {
                tagCount++;
            }
        }

        Logger.recordOutput(prefix + "tagCount", tagCount);

        // if (!usingConstrained && tagCount < 2) return;

        boolean reject = false;
        double meanDistance = 0;
        double meanAmbiguity = 0;
        double leastDistance = Double.MAX_VALUE;

        List<PhotonTrackedTarget> validTargets = result.getTargets().stream()
                .filter(t -> tagAllowed(t.getFiducialId()))
                .toList();

        if (validTargets.isEmpty()) return;

        for (var t : validTargets) {
            double dist = t.getBestCameraToTarget().getTranslation().getNorm();
            meanDistance += dist;
            leastDistance = Math.min(leastDistance, dist);
            meanAmbiguity += t.getPoseAmbiguity();
            if (t.getPoseAmbiguity() > 0.9) reject = true;
        }
        meanDistance /= validTargets.size();
        meanAmbiguity /= validTargets.size();

        if (meanDistance > maxMeanDistance) reject = true;

        if (!usingConstrained && meanAmbiguity > maxMeanAmbiguity) reject = true;

        if (Math.abs(pose.getRotation()
                .minus(RobotState.getInstance().getVisionPose().getRotation())
                .getRadians()) > 0.5) reject = true;

        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        if (Math.abs(omega) > maxYawRate) reject = true;

        Logger.recordOutput(prefix + "usingConstrained", usingConstrained);
        Logger.recordOutput(prefix + "rejected", reject);
        Logger.recordOutput(prefix + "rawPose", pose);

        if (!reject) {
            Vector<N3> distStdDevs;
            if (!usingConstrained) {
                distStdDevs = VecBuilder.fill(
                        Math.pow(meanDistance, 2) / 2.0,
                        Math.pow(meanDistance, 2) / 2.0,
                        0.0000001);
            } else {
                distStdDevs = VecBuilder.fill(
                        Math.pow(leastDistance, 2) / 4.0,
                        Math.pow(leastDistance, 2) / 4.0,
                        999999999);
            }
            RobotState.getInstance().addVisionData(List.of(
                    new RobotState.TimestampedVisionUpdate(
                            estimate.timestampSeconds, pose, distStdDevs)));
        }
    }

//     public void addVisionUpdateMt2(LimelightHelpers.PoseEstimate poseEstimate) {
//         String prefix = "Vision/" + name + "_mt2/";
//         double maxYawRate = Math.PI * 1.67; // 300 deg/s

// //        Logger.recordOutput(prefix + "heartbeat1", Math.random());

//         if (poseEstimate == null || poseEstimate.tagCount < 1) {
//             return;
//         }

// //        Logger.recordOutput(prefix + "heartbeat2", Math.random());
// //        Logger.recordOutput(prefix + "tagCount", poseEstimate.tagCount);

//         Pose2d pose = poseEstimate.pose;

//         if (turretMounted && SubsystemConstants.hasTurret) {
//             Logger.recordOutput(prefix + "untransformedPose", pose);
//             pose = transformTurretPose(pose, poseEstimate.timestampSeconds);
//         }

//         boolean reject = false;
//         double meanDistance = 0;
//         double leastDistance = 999999999;
//         for (var f : poseEstimate.rawFiducials) {
//             meanDistance += f.distToCamera;
//             leastDistance = Math.min(leastDistance, f.distToCamera);
//         }
//         meanDistance /= poseEstimate.tagCount;

//         if (Math.abs(
//                 pose.getRotation().minus(RobotState.getInstance().getVisionPose().getRotation()).getRadians()) > 0.5) {
//             reject = true;
//         }

// //        if (poseEstimate.timestampSeconds < resetOdometryTime + 0.5) {
// //            reject = true;
// //        }

//         double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
//         if (turretMounted && SubsystemConstants.hasTurret) {
//             omega -= RobotState.getInstance().getTurretVelocity();
//         }

//         if (turretMounted && RobotState.getInstance().getSpeed() > 1.08 ||  Math.abs(omega) > Math.PI*0.5) {
//             reject = true;
//         }

//         if (turretMounted &&
//                 Math.abs(pose.getRotation().getDegrees() - RobotState.getInstance().getVisionPose().getRotation().getDegrees()) > 16.7) {
//             reject = true;
//         }

//         if (Math.abs(omega) > maxYawRate) {
//             reject = true;
//         }

// //        Logger.recordOutput(prefix + "meanDistance", meanDistance);
// //        Logger.recordOutput(prefix + "leastDistance", leastDistance);
// //        Logger.recordOutput(prefix + "omega", omega);
//         Logger.recordOutput(prefix + "rejected", reject);
//         Logger.recordOutput(prefix + "rawPose", pose);

//         if (SubsystemConstants.ignoreTurretLimelight && turretMounted)
//             reject = true;

//         // if(turretMounted && RobotState.getInstance().getTurretVelocity() > 0.1)
//         // reject = true;

//         if (!reject) {
//             Vector<N3> distStdDevs;
//             distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 4.0, Math.pow(leastDistance, 2) / 4.0,
//                     999999999);
//             RobotState.TimestampedVisionUpdate update = new RobotState.TimestampedVisionUpdate(
//                     poseEstimate.timestampSeconds, pose, distStdDevs);

//             RobotState.getInstance().addVisionData(List.of(update));
//         }
//     }
    
    public void updatePose() {
        updateSide();
        for (var result : camera.getAllUnreadResults()) {
            addVisionUpdate(result);
            // addVisionUpdateMt2(result);
        }
    }
}
