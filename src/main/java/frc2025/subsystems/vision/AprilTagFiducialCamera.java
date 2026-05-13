
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

   private void addVisionUpdateMultiTag(PhotonPipelineResult result) {
        String prefix = "Vision/" + name + "_mt1/";

        double maxMeanDistance = 3.5;
        double maxMeanAmbiguity = 0.2;
        double maxYawRate = Math.PI * 1.67;



        if (!result.hasTargets()) return;

        Optional<EstimatedRobotPose> optionalPose = multiTagEstimator.estimateCoprocMultiTagPose(result);

        Logger.recordOutput(prefix + "isPresent", optionalPose.isPresent());
        if (optionalPose.isPresent()) {
            Logger.recordOutput(prefix + "optionalPose", optionalPose.get().estimatedPose.toPose2d());
        }
            

        if (optionalPose.isEmpty()) return;


        int allowedTagsUsed = (int) optionalPose.get().targetsUsed.stream().filter(t -> tagAllowed(t.getFiducialId())).count();

        if (allowedTagsUsed < 2) return;

        EstimatedRobotPose estimate = optionalPose.get();
        Pose2d pose = estimate.estimatedPose.toPose2d();


        List<PhotonTrackedTarget> usedTargets = estimate.targetsUsed.stream().filter(t -> tagAllowed(t.getFiducialId())).toList();

        boolean reject = false;
        double meanDistance = 0;
        double meanAmbiguity = 0;

        for (var t : usedTargets) {
            double dist = t.getBestCameraToTarget().getTranslation().getNorm();
            meanDistance += dist;
            meanAmbiguity += t.getPoseAmbiguity();
            if (t.getPoseAmbiguity() > 0.9) reject = true;
        }
        meanDistance /= usedTargets.size();
        meanAmbiguity /= usedTargets.size();

        if (meanDistance > maxMeanDistance) reject = true;
        if (meanAmbiguity > maxMeanAmbiguity) reject = true;

        if (Math.abs(pose.getRotation()
                .minus(RobotState.getInstance().getVisionPose().getRotation())
                .getRadians()) > 0.5) reject = true;

        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        if (Math.abs(omega) > maxYawRate) reject = true;

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

        if (!reject) {
            Vector<N3> distStdDevs = VecBuilder.fill(
                    Math.pow(meanDistance, 2) / 2.0,
                    Math.pow(meanDistance, 2) / 2.0,
                    0.0000001);
            RobotState.getInstance().addVisionData(List.of(
                    new RobotState.TimestampedVisionUpdate(
                            estimate.timestampSeconds, pose, distStdDevs)));
        }
    }

    private void addVisionUpdateConstrained(PhotonPipelineResult result) {
        String prefix = "Vision/" + name + "_mt2/";

        double maxMeanDistance = 3.5;
        double maxYawRate = Math.PI * 1.67;

        if (!result.hasTargets()) return;

        var calibData = camera.getCameraMatrix();
        var distCoeffs = camera.getDistCoeffs();
        if (calibData.isEmpty() || distCoeffs.isEmpty()) return;

        Pose3d seedPose = new Pose3d(RobotState.getInstance().getVisionPose());
        Optional<EstimatedRobotPose> optionalPose = constrainedPnPEstimator.estimateConstrainedSolvepnpPose(
                result,
                calibData.get(),
                distCoeffs.get(),
                seedPose,
                false,
                0.5
        );

        Logger.recordOutput(prefix + "isPresent", optionalPose.isPresent());
        if (optionalPose.isPresent()) {
            Logger.recordOutput(prefix + "optionalPose", optionalPose.get().estimatedPose.toPose2d());
        }

        if (optionalPose.isEmpty()) return;

        EstimatedRobotPose estimate = optionalPose.get();
        Pose2d pose = estimate.estimatedPose.toPose2d();

        List<PhotonTrackedTarget> usedTargets = estimate.targetsUsed.stream().filter(t -> tagAllowed(t.getFiducialId())).toList();

        if (usedTargets.isEmpty()) return;

        boolean reject = false;
        double meanDistance = 0;
        double leastDistance = Double.MAX_VALUE;

        for (var t : usedTargets) {
            double dist = t.getBestCameraToTarget().getTranslation().getNorm();
            meanDistance += dist;
            leastDistance = Math.min(leastDistance, dist);
        }
        meanDistance /= usedTargets.size();

        if (meanDistance > maxMeanDistance) reject = true;


        if (Math.abs(pose.getRotation()
                .minus(RobotState.getInstance().getVisionPose().getRotation())
                .getRadians()) > 0.5) reject = true;

        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        if (Math.abs(omega) > maxYawRate) reject = true;

        Logger.recordOutput(prefix + "tagCount", usedTargets.size());
        Logger.recordOutput(prefix + "rejected", reject);
        Logger.recordOutput(prefix + "rawPose", pose);


        Logger.recordOutput(prefix + "meanDistance", meanDistance);
        // Logger.recordOutput(prefix + "meanAmbiguity", meanAmbiguity); // MT1 only

        Logger.recordOutput(prefix + "rotationDelta", Math.abs(pose.getRotation()
            .minus(RobotState.getInstance().getVisionPose().getRotation())
            .getRadians()));
        Logger.recordOutput(prefix + "omega", RobotState.getInstance()
            .getFieldRelativeSpeeds().omegaRadiansPerSecond);
        Logger.recordOutput(prefix + "visionPoseRotation", 
            RobotState.getInstance().getVisionPose().getRotation().getDegrees());
        Logger.recordOutput(prefix + "estimatedRotation",
            pose.getRotation().getDegrees());

        if (!reject) {
            Vector<N3> distStdDevs = VecBuilder.fill(
                    Math.pow(leastDistance, 2) / 4.0,
                    Math.pow(leastDistance, 2) / 4.0,
                    999999999);
            RobotState.getInstance().addVisionData(List.of(
                    new RobotState.TimestampedVisionUpdate(
                            estimate.timestampSeconds, pose, distStdDevs)));
        }
    }

    public void updatePose() {
        updateSide();
        for (var result : camera.getAllUnreadResults()) {
            addVisionUpdateMultiTag(result);
            addVisionUpdateConstrained(result);
        }
    }
}