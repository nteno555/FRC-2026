
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
        LimelightHelpers.SetIMUMode(name, 0);
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

    public void addVisionUpdate(LimelightHelpers.PoseEstimate poseEstimate, boolean mt2) {
        if(poseEstimate == null) return;
        if(poseEstimate.tagCount == 0) return;

        Pose2d pose = poseEstimate.pose;
        Pose2d robotToCameraPose = RobotState.getInstance().getTurretLimelightPose();

//        if(turretMounted){// && robotToCameraPose != null) {
//            Transform2d robotToShooterReal = new Transform2d(
//                    SubsystemConstants.robotToShooter.getTranslation(),
//                    SubsystemConstants.robotToShooter.getRotation().plus(Rotation2d.fromRadians(RobotState.getInstance().getTurretAngle()))
//            );
//
//            Transform2d cameraToRobotTransform = SubsystemConstants.shooterToCamera.inverse().plus(robotToShooterReal.inverse());
//            Transform2d robotToCameraTransform = cameraToRobotTransform.inverse();
//
//            Logger.recordOutput("Vision/turretMountedHeartbeat", Math.random());
//
//            LimelightHelpers.setCameraPose_RobotSpace(name, robotToCameraTransform.getX(), robotToCameraTransform.getY(), 0.39,
//                    0.0, 29.1, robotToCameraTransform.getRotation().getDegrees());
////            pose = pose.transformBy(cameraToRobotTransform);
//        } else {
//            Transform2d cameraToRobotTransform = SubsystemConstants.robotToFrontCamera.inverse();
//
//            LimelightHelpers.setCameraPose_RobotSpace(name, 0.0, 0.0, 0.1,
//                    0.0, 18.0, 0.0);
////            pose = pose.transformBy(cameraToRobotTransform);
//        }

        double meanAmbiguity = 0.0;
        int tagCount = 0;
        double leastDistance = Double.MAX_VALUE;
        double leastAmbiguity = Double.MAX_VALUE;
        for (LimelightHelpers.RawFiducial f : poseEstimate.rawFiducials) {
            if (f.distToRobot < leastDistance) leastDistance = f.distToRobot;
            if (f.ambiguity < leastAmbiguity) leastAmbiguity = f.ambiguity;
            meanAmbiguity += f.ambiguity;
            tagCount += 1;
        }

        meanAmbiguity /= tagCount;

        double ambiguityThreshold = 0.5;
        boolean reject = false;
        if (leastAmbiguity > ambiguityThreshold) {
            reject = true;
        }

        if (Math.abs(pose.getRotation().minus(RobotState.getInstance().getVisionPose().getRotation()).getRadians()) > 0.5) reject = true;

        Vector<N3> distStdDevs;

        double rotationStdDev = 99999999;
        if(!mt2) rotationStdDev = Math.pow(leastDistance, 2) / 10000.0;
        distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 2.0, Math.pow(leastDistance, 2) / 2.0, rotationStdDev);

        VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
                poseEstimate.timestampSeconds, pose, distStdDevs
        );

        Logger.recordOutput("Vision/VisionPose_" + name + "_raw" + (mt2 ? "_mt2" : "_mt1"), poseEstimate.pose);
        Logger.recordOutput("Vision/VisionPose_" + name + (mt2 ? "_mt2" : "_mt1"), pose);

        if(!reject) {
            Logger.recordOutput("Vision/notRejectedHeartbeat_" + name, Math.random());

            if (!turretMounted) {
                RobotState.getInstance().updatePathFollowingPose(List.of(update));
            }
            else {
                RobotState.getInstance().addVisionData(List.of(update));
            }
        }
    }

    public void updatePose() {
        updateSide();

        LimelightHelpers.LimelightTarget_Fiducial fiducial = new LimelightHelpers.LimelightTarget_Fiducial();
        fiducial.ta = LimelightHelpers.getTA(name);
        fiducial.fiducialID = LimelightHelpers.getFiducialID(name);

//        boolean usingMt1 = true;
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        LimelightHelpers.PoseEstimate poseEstimateMt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        Logger.recordOutput("Vision/heartbeatBeforeNullcheck", Math.random());

        addVisionUpdate(poseEstimate, false);
        addVisionUpdate(poseEstimateMt2, true);
    }
}
