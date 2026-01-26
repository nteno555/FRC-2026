
package frc2025.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc2025.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class LimelightFiducialCamera {
    String name;
    public LimelightFiducialCamera(String name) {
        this.name = name;
    }
    public void resetGyro() {}
    public void setUseGyro() {}
    public void updateReefAlign() {}
    public void writeGyro() {
        LimelightHelpers.SetIMUMode(name, 0);
        LimelightHelpers.SetRobotOrientation(name, RobotState.getInstance().getVisionPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }
    public void updateSide() {
        /*if(!Alliance.get()) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{17, 18, 19, 20, 21, 22});
        } else {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{6, 7, 8, 9, 10, 11});
        }*/
    }
    public void updatePose() {
        updateSide();

        LimelightHelpers.LimelightTarget_Fiducial fiducial = new LimelightHelpers.LimelightTarget_Fiducial();
        fiducial.ta = LimelightHelpers.getTA(name);
        fiducial.fiducialID = LimelightHelpers.getFiducialID(name);

        boolean usingMt1 = true;
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        //LimelightHelpers.PoseEstimate poseEstimateMt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        Logger.recordOutput("Vision/heartbeatBeforeNullcheck", Math.random());

        if(poseEstimate == null) return;
        if(poseEstimate.tagCount == 0) return;

        Logger.recordOutput("Vision/mt1Pose", poseEstimate.pose);

//        if(poseEstimate.avgTagDist > 2) {
//            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
//            usingMt1 = false;
//        }


        Pose2d pose = poseEstimate.pose;

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

        if (Math.abs(pose.getRotation().minus(RobotState.getInstance().getVisionPose().getRotation()).getRadians()) > 1.0) reject = true;

        Vector<N3> distStdDevs;

        if(usingMt1) {
            distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 5.0, Math.pow(leastDistance, 2) / 5.0, Math.pow(leastDistance, 2) / 10000.0);
            //distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 5.0, Math.pow(leastDistance, 2) / 5.0, 999999);
        } else {
            distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 5.0, Math.pow(leastDistance, 2) / 5.0, 999999);
        }

        VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
                poseEstimate.timestampSeconds, pose, distStdDevs
        );
        Logger.recordOutput("Vision/VisionPose_" + name, poseEstimate.pose);

        if(!reject) {
            Logger.recordOutput("Vision/notRejectedHeartbeat_" + name, Math.random());
            RobotState.getInstance().addVisionData(List.of(update));
        }
    }
}
