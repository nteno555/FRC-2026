
package frc2025.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveConstants;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class LimelightFiducialCamera {
    private final String name;
    private final boolean turretMounted;
    private final double robotToCameraUp;
    private final double robotToCameraForward;
    private final double robotToCameraRight;
    private final double robotToCameraYaw;
    private final double robotToCameraPitch;
    private final double robotToCameraRoll;

    public LimelightFiducialCamera(String name, boolean turretMounted, double robotToCameraForward, double robotToCameraRight, double robotToCameraUp,
                                  double robotToCameraYaw, double robotToCameraPitch, double robotToCameraRoll) {
        this.name = name;
        this.turretMounted = turretMounted;

        this.robotToCameraForward = robotToCameraForward;
        this.robotToCameraRight = robotToCameraRight;
        this.robotToCameraUp = robotToCameraUp;
        this.robotToCameraYaw = robotToCameraYaw;
        this.robotToCameraPitch = robotToCameraPitch;
        this.robotToCameraRoll = robotToCameraRoll;
    }

    public void resetOdometry() {
        LimelightHelpers.SetIMUMode(name, 1);
    }

    public void updateReefAlign() {}

    public void writeGyro() {
        String prefix = "Vision/" + name + "_mt2/";

        LimelightHelpers.SetIMUMode(name, 1);
//        LimelightHelpers.SetIMUAssistAlpha(name, 0.1);
//        LimelightHelpers.SetIMUMode(name, 0);

        if(turretMounted){// && robotToCameraPose != null) {
//            Transform2d robotToShooterReal = new Transform2d(
//                    SubsystemConstants.robotToShooter.getTranslation(),
//                    SubsystemConstants.robotToShooter.getRotation().plus(Rotation2d.fromRadians(RobotState.getInstance().getTurretAngle()))
//            );
//
//            Transform2d cameraToRobotTransform = SubsystemConstants.shooterToCamera.inverse().plus(robotToShooterReal.inverse());
//            Transform2d robotToCameraTransform = cameraToRobotTransform.inverse();

            LimelightHelpers.setCameraPose_RobotSpace(name, 0.0, 0.0, this.robotToCameraUp,
                    this.robotToCameraRoll, this.robotToCameraPitch, 0.0);

            double orientation =  RobotState.getInstance().getVisionPose().getRotation().getDegrees() - Units.radiansToDegrees(RobotState.getInstance().getTurretAngle());

            double omega = RobotState.getInstance().getTurretVelocity() - RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
            omega = Units.radiansToDegrees(omega);

            LimelightHelpers.SetRobotOrientation(name, orientation, omega, 0, 0, 0, 0);

            Logger.recordOutput(prefix + "mt2RobotOrientation", orientation);
            Logger.recordOutput(prefix + "mt2UpdateGyroHeartbeat", Math.random());
        } else {
            LimelightHelpers.setCameraPose_RobotSpace(name, this.robotToCameraForward, this.robotToCameraRight, this.robotToCameraUp,
                    this.robotToCameraRoll, this.robotToCameraPitch, this.robotToCameraYaw);

            double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
            omega = Units.radiansToDegrees(omega);

            LimelightHelpers.SetRobotOrientation(name, RobotState.getInstance().getVisionPose().getRotation().getDegrees(), omega, 0, 0, 0, 0);
            Logger.recordOutput(prefix + "mt2UpdateGyroHeartbeat", Math.random());
        }
    }

    public void updateSide() {
        if(Alliance.get()) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{2, 3, 4, 5, 8, 9, 10, 11, 25, 26});
        } else {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{9, 10, 18, 19, 20, 21, 24, 25, 26, 27});
        }
    }

    public void toggleClimbTags(boolean climbing) {
        if (climbing) {
            if (Alliance.get()) {
                LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{15, 16});
            }
            else {
                LimelightHelpers.SetFiducialIDFiltersOverride(name,  new int[]{31, 32});
            }
        }
        else {
            if(Alliance.get()) {
                LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{2, 3, 4, 5, 8, 9, 10, 11, 25, 26});
            } else {
                LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{9, 10, 18, 19, 20, 21, 24, 25, 26, 27});
            }
        }
    }

    private Pose2d transformTurretPose(Pose2d pose) {
        if(SubsystemConstants.hasTurret) {
            Transform2d robotToShooterReal = new Transform2d(
                    SubsystemConstants.robotToShooter.getTranslation(),
                    SubsystemConstants.robotToShooter.getRotation().plus(Rotation2d.fromRadians(-RobotState.getInstance().getTurretAngle()))
            );

            //        Transform2d cameraToRobotTransform = SubsystemConstants.shooterToCamera.inverse().plus(robotToShooterReal.inverse());
            Transform2d cameraToRobotTransform = robotToShooterReal.plus(SubsystemConstants.shooterToCamera).inverse();

            Logger.recordOutput("Vision/" + name + "_mt2/cameraPose", RobotState.getInstance().getVisionPose().transformBy(cameraToRobotTransform.inverse()));

            return pose.transformBy(cameraToRobotTransform);
        } else {
            Transform2d cameraToRobotTransform = SubsystemConstants.robotToShooter.plus(SubsystemConstants.shooterToCamera).inverse();
            return pose.transformBy(cameraToRobotTransform);
        }
    }

    public void addVisionUpdateMt1(LimelightHelpers.PoseEstimate poseEstimate) {
        String prefix = "Vision/" + name + "_mt1/";

        double maxMeanDistance = 2.5;
        double maxMeanAmbiguity = 0.2;
        double minMeanArea = 0.0;
        double maxYawRate = Math.PI * 2;

        Logger.recordOutput(prefix + "heartbeat1", Math.random());

        if(poseEstimate == null) return;

        Logger.recordOutput(prefix + "heartbeat2", Math.random());
        Logger.recordOutput(prefix + "tagCount", poseEstimate.tagCount);

        boolean reject = false;

        if(poseEstimate.tagCount < 2) {
            reject = true;
        }

        Pose2d pose = poseEstimate.pose;

        if(turretMounted && SubsystemConstants.hasTurret) {
            Logger.recordOutput(prefix + "untransformedPose", pose);
            pose = transformTurretPose(pose);
        }

        double meanDistance = 0;
        double meanAmbiguity = 0;
        double meanArea = 0;
        for(var f: poseEstimate.rawFiducials) {
            meanDistance += f.distToCamera;
            meanAmbiguity += f.ambiguity;
            meanArea += f.ta;

            if(f.ambiguity > 0.9) reject = true;
        }
        meanDistance /= poseEstimate.tagCount;
        meanAmbiguity /= poseEstimate.tagCount;
        meanArea /= poseEstimate.tagCount;


        if(meanAmbiguity > maxMeanAmbiguity || meanDistance > maxMeanDistance || meanArea < minMeanArea) {
            reject = true;
        }
        if (Math.abs(pose.getRotation().minus(RobotState.getInstance().getVisionPose().getRotation()).getRadians()) > 0.5) {
            reject = true;
        }

        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        if(turretMounted && SubsystemConstants.hasTurret) {
            omega -= RobotState.getInstance().getTurretVelocity();
        }
        if(Math.abs(omega) > maxYawRate) {
            reject = true;
        }

        Logger.recordOutput(prefix + "meanDistance", meanDistance);
        Logger.recordOutput(prefix + "meanAmbiguity", meanAmbiguity);
        Logger.recordOutput(prefix + "meanArea", meanArea);
        Logger.recordOutput(prefix + "rejected", reject);
        Logger.recordOutput(prefix + "rawPose", pose);

        // if(SubsystemConstants.ignoreTurretLimelight && turretMounted)
        //     reject = true;

        if(!reject) {
            Vector<N3> distStdDevs;
            if(!turretMounted) {
                distStdDevs = VecBuilder.fill(Math.pow(meanDistance, 2) / 2.0, Math.pow(meanDistance, 2) / 2.0, 0.0000001);
            }
            else {
                distStdDevs = VecBuilder.fill(Math.pow(meanDistance, 2) / 2.0, Math.pow(meanDistance, 2) / 2.0, 99999999999.0);
            }

            VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
                    poseEstimate.timestampSeconds, pose, distStdDevs
            );

            RobotState.getInstance().addVisionData(List.of(update));
        }
    }

    public void addVisionUpdateMt2(LimelightHelpers.PoseEstimate poseEstimate) {
        String prefix = "Vision/" + name + "_mt2/";
        double maxYawRate = Math.PI * 2;

        Logger.recordOutput(prefix + "heartbeat1", Math.random());

        if(poseEstimate == null || poseEstimate.tagCount < 1) return;

        Logger.recordOutput(prefix + "heartbeat2", Math.random());
        Logger.recordOutput(prefix + "tagCount", poseEstimate.tagCount);

        Pose2d pose = poseEstimate.pose;

        if(turretMounted && SubsystemConstants.hasTurret) {
            Logger.recordOutput(prefix + "untransformedPose", pose);
            pose = transformTurretPose(pose);
        }

        boolean reject = false;
        double meanDistance = 0;
        double leastDistance = 999999999;
        for(var f: poseEstimate.rawFiducials) {
            meanDistance += f.distToCamera;
            leastDistance = Math.min(leastDistance, f.distToCamera);
        }
        meanDistance /= poseEstimate.tagCount;


        if (Math.abs(pose.getRotation().minus(RobotState.getInstance().getVisionPose().getRotation()).getRadians()) > 0.5) {
            reject = true;
        }

        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        if(turretMounted && SubsystemConstants.hasTurret) {
            omega += RobotState.getInstance().getTurretVelocity();
        }

        if(Math.abs(omega) > maxYawRate) {
            reject = true;
        }

        Logger.recordOutput(prefix + "meanDistance", meanDistance);
        Logger.recordOutput(prefix + "leastDistance", leastDistance);
        Logger.recordOutput(prefix + "omega", omega);
        Logger.recordOutput(prefix + "rejected", reject);
        Logger.recordOutput(prefix + "rawPose", pose);

        // if(SubsystemConstants.ignoreTurretLimelight && turretMounted)
        //     reject = true;

        if(!reject) {
            Vector<N3> distStdDevs;
            distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 4.0, Math.pow(leastDistance, 2) / 4.0, 999999999);
            VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
                    poseEstimate.timestampSeconds, pose, distStdDevs
            );

            RobotState.getInstance().addVisionData(List.of(update));
        }
    }

    public void updatePose() {
        updateSide();

        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        LimelightHelpers.PoseEstimate poseEstimateMt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        addVisionUpdateMt1(poseEstimate);
        addVisionUpdateMt2(poseEstimateMt2);
    }
}