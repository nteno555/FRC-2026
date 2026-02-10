
package frc2025.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class LimelightFiducialCamera {
    String name;
    boolean turretMounted;

    public LimelightFiducialCamera(String name, boolean turretMounted) {
        this.name = name;
        this.turretMounted = turretMounted;
    }

    public void resetGyro() {}

    public void setUseGyro() {}

    public void updateReefAlign() {}

    public void writeGyro() {
        LimelightHelpers.SetIMUMode(name, 4);
        LimelightHelpers.SetRobotOrientation(name, RobotState.getInstance().getVisionPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        if(turretMounted){// && robotToCameraPose != null) {
            Transform2d robotToShooterReal = new Transform2d(
                    SubsystemConstants.robotToShooter.getTranslation(),
                    SubsystemConstants.robotToShooter.getRotation().plus(Rotation2d.fromRadians(RobotState.getInstance().getTurretAngle()))
            );

            Transform2d cameraToRobotTransform = SubsystemConstants.shooterToCamera.inverse().plus(robotToShooterReal.inverse());
            Transform2d robotToCameraTransform = cameraToRobotTransform.inverse();

            Logger.recordOutput("Vision/turretMountedHeartbeat", Math.random());

            LimelightHelpers.setCameraPose_RobotSpace(name, robotToCameraTransform.getX(), robotToCameraTransform.getY(), 0.39,
                    0.0, 29.1, robotToCameraTransform.getRotation().getDegrees());
//            pose = pose.transformBy(cameraToRobotTransform);
        } else {
            Transform2d cameraToRobotTransform = SubsystemConstants.robotToFrontCamera.inverse();

            LimelightHelpers.setCameraPose_RobotSpace(name, 0.0, 0.0, 0.1,
                    0.0, 18.0, 0.0);
//            pose = pose.transformBy(cameraToRobotTransform);
        }
    }

    public void updateSide() {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{2, 3, 4, 5, 8, 9, 10, 11});
        } else {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{17, 18, 19, 20, 21, 24, 25, 26, 27, 28});
        }
    }

    public void addVisionUpdateMt1(LimelightHelpers.PoseEstimate poseEstimate) {
        String prefix = "Vision/" + name + "_mt1/";

        double maxMeanDistance = 2.5;
        double maxMeanAmbiguity = 0.5;
        double minMeanArea = 0.1;
        double maxYawRate = Math.PI * 2;

        if(poseEstimate == null || poseEstimate.tagCount < 2) return;

        boolean reject = false;
        Pose2d pose = poseEstimate.pose;
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
        if(turretMounted) {
            omega += RobotState.getInstance().getTurretVelocity();
        }
        if(omega > maxYawRate) {
            reject = true;
        }

        Logger.recordOutput(prefix + "meanDistance", meanDistance);
        Logger.recordOutput(prefix + "meanAmbiguity", meanAmbiguity);
        Logger.recordOutput(prefix + "meanArea", meanArea);
        Logger.recordOutput(prefix + "rejected", reject);
        Logger.recordOutput(prefix + "rawPose", pose);

        if(!reject) {
            Vector<N3> distStdDevs;
            distStdDevs = VecBuilder.fill(Math.pow(meanDistance, 2) / 5.0, Math.pow(meanDistance, 2) / 5.0, Math.pow(meanDistance, 2) / 100.0); // maybe use least distance instead
            VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
                    poseEstimate.timestampSeconds, pose, distStdDevs
            );

            RobotState.getInstance().addVisionData(List.of(update));
        }
    }

    public void addVisionUpdateMt2(LimelightHelpers.PoseEstimate poseEstimate) {
        String prefix = "Vision/" + name + "_mt2/";
        double maxYawRate = Math.PI * 2;

        if(poseEstimate == null) return;

        boolean reject = false;
        Pose2d pose = poseEstimate.pose;
        double meanDistance = 0;
        for(var f: poseEstimate.rawFiducials) {
            meanDistance += f.distToCamera;
        }
        meanDistance /= poseEstimate.tagCount;


        if (Math.abs(pose.getRotation().minus(RobotState.getInstance().getVisionPose().getRotation()).getRadians()) > 0.5) {
            reject = true;
        }

        double omega = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;
        if(turretMounted) {
            omega += RobotState.getInstance().getTurretVelocity();
        }

        if(omega > maxYawRate) {
            reject = true;
        }

        Logger.recordOutput(prefix + "meanDistance", meanDistance);
        Logger.recordOutput(prefix + "omega", omega);
        Logger.recordOutput(prefix + "rejected", reject);
        Logger.recordOutput(prefix + "rawPose", pose);

        if(!reject) {
            Vector<N3> distStdDevs;
            distStdDevs = VecBuilder.fill(Math.pow(meanDistance, 2) / 5.0, Math.pow(meanDistance, 2) / 5.0, 999999999); // maybe use least distance instead
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
