//package frc2025.subsystems.vision;
//
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.Vector;
//import edu.wpi.first.math.geometry.*;
//import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.wpilibj.DriverStation;
//import frc2025.subsystems.vision.LimelightHelpers;
//
//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//
//import java.util.List;
//import java.util.Map;
//import java.util.Set;
//
//
//public class LimelightFiducialCameraReef extends LimelightFiducialCamera{
//    private Map<Integer, Double> blueReefTags;
//    private List<Integer> reefTags;
//
//    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
//    Transform2d robotToCamera;
//
//    private boolean autoDisabled = true;
//
//    public LimelightFiducialCameraReef(String limelightName) {
//        name = limelightName;
//    }
//
//    public void updateSide() {
//
//        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
//            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{17, 18, 19, 20, 21, 22});
//            reefTags = List.of(17, 18, 19, 20, 21, 22);
//            blueReefTags = Map.of(
//                    18, 0.0+360,
//                    19, -60.0+360,
//                    20, -120.0+360,
//                    21, -180.0+360,
//                    22, -240.0+360,
//                    17, -300.0+360
//            );
//        } else {
//            LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{6, 7, 8, 9, 10, 11});
//            reefTags = List.of(6, 7, 8, 9, 10, 11);
//            blueReefTags = Map.of(
//                    7, 0.0+180,
//                    8, 60.0+180,
//                    9, 120.0+180,
//                    10, 180.0+180,
//                    11, 240.0+180,
//                    6, 300.0+180
//            );
//        }
//    }
//
////        @Override
////    public Pose2d updatePose(RobotState state, Commands commands) {
//////        if(state.gamePeriod == RobotState.GamePeriod.DISABLED && autoDisabled) {
//////            resetGyro(state);
//////        } else if(state.gamePeriod == RobotState.GamePeriod.AUTO) {
//////            autoDisabled = false;
//////        }
////
////        updateSide();
////
////        LimelightHelpers.LimelightTarget_Fiducial fiducial = new LimelightHelpers.LimelightTarget_Fiducial();
////        fiducial.ta = LimelightHelpers.getTA(name);
////        fiducial.fiducialID = LimelightHelpers.getFiducialID(name);
////
////        if (blueReefTags.containsKey((int) fiducial.fiducialID)) state.reefVisionResults.put(name, fiducial);
////
////        LimelightHelpers.PoseEstimate poseEstimate;
////        //poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
////        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
////
////        if(state.minTagDist < 0.5) {
////            LiveGraph.add("Vision/angleCorrection", true);
////            var poseEstimateMt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
////            if(poseEstimateMt1 != null) {
////                Rotation2d newRotation = LimelightHelpers.getBotPoseEstimate_wpiBlue(name).pose.getRotation();
////                //state.visionPoseEstimator.resetRotation(newRotation);
////            }
////            //poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
////            //correctGyro(state.visionPoseEstimator.getLatestPose().getRotation().getDegrees());
////            //poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
////        } else {
////            LiveGraph.add("Vision/angleCorrection", false);
////            //poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
////            //poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
////        }
////
////
////        if(poseEstimate == null) return null;
////        if(poseEstimate.tagCount == 0) return null;
////        if(state.swervePoseMeters == null) return null;
////
////        Pose2d pose = poseEstimate.pose;
////
////        double meanAmbiguity = 0.0;
////        int tagCount = 0;
////        double leastDistance = Double.MAX_VALUE;
////        double leastAmbiguity = Double.MAX_VALUE;
////        for (LimelightHelpers.RawFiducial f : poseEstimate.rawFiducials) {
////            if (f.distToRobot < leastDistance) leastDistance = f.distToRobot;
////            if (f.ambiguity < leastAmbiguity) leastAmbiguity = f.ambiguity;
////            meanAmbiguity += f.ambiguity;
////            tagCount += 1;
////        }
////
////        meanAmbiguity /= tagCount;
////
////        double ambiguityThreshold = 0.2;
////        boolean reject = false;
////        if (leastAmbiguity > ambiguityThreshold) {
////            reject = true;
////        }
////        if(Math.abs(poseEstimate.pose.getRotation().minus(state.swervePoseMeters.getRotation()).getRadians()) > 0.3)
////            reject = true;
////
////        Vector<N3> distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 6.0, Math.pow(leastDistance, 2) / 6.0, 9999999);
////        //Vector<N3> distStdDevs = VecBuilder.fill(0.1, 0.1, 999999);
////
////        VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
////                poseEstimate.timestampSeconds, pose, distStdDevs
////        );
////        LiveGraph.add("Vision/ReefAlignPose_" + name, poseEstimate.pose);
////
////        //if(state.gamePeriod != RobotState.GamePeriod.AUTO || (Math.abs(state.pigeonAngularVelocity) < 720 && state.hasPiece)) {
////        //if(state.gamePeriod != RobotState.GamePeriod.AUTO && Math.abs(state.pigeonAngularVelocity) < 720) {
////        //reject = false;
////        if(Math.abs(state.pigeonAngularVelocity) < 720) {
////            if (!reject) state.visionPoseEstimator.addVisionData(List.of(update));
////            //state.visionPoseEstimator.resetPose(poseEstimate.pose);
////        }
////
////        return poseEstimate.pose;
////    }
//
//    Rotation2d lastGyroAngle = new Rotation2d();
//
//    public void updateRotationCorrection(RobotState state, Commands commands) {
//        var mt1PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
//        Rotation2d rotation = new Rotation2d();
//        if (state.swervePoseMeters != null)
//            rotation = state.swervePoseMeters.getRotation();
//
//        state.shouldResetRotation = false;
//        /*
//        if (mt1PoseEstimate == null || state.rotationReset) {
//            state.mt1Rotation = state.mt1Rotation.plus(rotation.minus(lastGyroAngle));
//            state.rotationReset = false;
//        } else if ((mt1PoseEstimate.avgTagDist > 1 || mt1PoseEstimate.pose.getRotation().getDegrees() == 0) || state.rotationReset ||
//                (mt1PoseEstimate.pose.getRotation().minus(rotation).getRadians()) > 0.5) {
//            state.mt1Rotation = state.mt1Rotation.plus(rotation.minus(lastGyroAngle));
//            state.rotationReset = false;
//        } else{
//            state.mt1Rotation = mt1PoseEstimate.pose.getRotation();
//            state.shouldResetRotation = true;
//        }
//         */
//        if (mt1PoseEstimate != null) {
//            if ((Math.abs(mt1PoseEstimate.pose.getRotation().minus(rotation).getRadians()) < 0.35 || Math.abs(mt1PoseEstimate.pose.getRotation().minus(rotation).getRadians()) > 2*Math.PI - 0.35) &&
//                    mt1PoseEstimate.avgTagDist < 1.5) {
//                state.mt1Rotation = mt1PoseEstimate.pose.getRotation();
//            } else {
//                state.mt1Rotation = rotation;
//            }
//            /*if(Math.abs(mt1PoseEstimate.pose.getRotation().minus(rotation).getDegrees()) < 15) {
//                state.mt1Rotation = mt1PoseEstimate.pose.getRotation();
//            } else {
//                state.mt1Rotation = rotation;
//            }*/
//        } else {
//            state.mt1Rotation = rotation;
//        }
//        if(mt1PoseEstimate != null) {
//            LiveGraph.add("Vision/mt1Pose", mt1PoseEstimate.pose);
//            LiveGraph.add("Vision/mt1RotationReal", mt1PoseEstimate.pose.getRotation());
//        }
//        LiveGraph.add("Vision/mt1Rotation", state.mt1Rotation.getDegrees());
//        LiveGraph.add("Vision/mt1RotationEstimate", state.mt1RotationEstimator.get());
//        lastGyroAngle = rotation;
//    }
//
//    public boolean tagVisible() {
//        if (blueReefTags == null) {
//            return false;
//        }
//        return blueReefTags.containsKey((int) LimelightHelpers.getFiducialID(name));
//    }
//    @Override
//    public Pose2d updatePose(RobotState state, Commands commands) {
////        if(state.gamePeriod == RobotState.GamePeriod.DISABLED && autoDisabled) {
////            resetGyro(state);
////        } else if(state.gamePeriod == RobotState.GamePeriod.AUTO) {
////            autoDisabled = false;
////        }
//
//        updateSide();
//        if(name.equals("limelight-xleft") && !commands.getWantLeftLimelight())
//            updateRotationCorrection(state, commands);
//        if(name.equals("limelight-xright") && commands.getWantLeftLimelight())
//            updateRotationCorrection(state, commands);
//
//        LimelightHelpers.LimelightTarget_Fiducial fiducial = new LimelightHelpers.LimelightTarget_Fiducial();
//        fiducial.ta = LimelightHelpers.getTA(name);
//        fiducial.fiducialID = LimelightHelpers.getFiducialID(name);
//
//        if (blueReefTags.containsKey((int) fiducial.fiducialID)) state.reefVisionResults.put(name, fiducial);
//
//        LimelightHelpers.PoseEstimate poseEstimate;
//        //if(!state.reefAligning)
//        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
//        //else
//        //    poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
//
//        if(poseEstimate == null) return null;
//        if(poseEstimate.tagCount == 0) return null;
//        if(state.swervePoseMeters == null) return null;
//
//        //Pose2d pose = new Pose2d(poseEstimate.pose.getTranslation(), state.swervePoseMeters.getRotation());
//        Pose2d pose = poseEstimate.pose;
//
//        double meanAmbiguity = 0.0;
//        int tagCount = 0;
//        double leastDistance = Double.MAX_VALUE;
//        double leastAmbiguity = Double.MAX_VALUE;
//        for (LimelightHelpers.RawFiducial f : poseEstimate.rawFiducials) {
//            if (f.distToRobot < leastDistance) leastDistance = f.distToRobot;
//            if (f.ambiguity < leastAmbiguity) leastAmbiguity = f.ambiguity;
//            meanAmbiguity += f.ambiguity;
//            tagCount += 1;
//        }
//
//        meanAmbiguity /= tagCount;
//
//        double ambiguityThreshold = 0.5;
//        boolean reject = false;
//        if (leastAmbiguity > ambiguityThreshold) {
//            reject = true;
//        }
//
//        if (Math.abs(pose.getRotation().minus(state.visionPoseEstimator.getLatestPose().getRotation()).getRadians()) > 0.3) reject = true;
//
//        Vector<N3> distStdDevs = VecBuilder.fill(Math.pow(leastDistance, 2) / 4.0, Math.pow(leastDistance, 2) / 4.0, 0.2);
//
//        VisionPoseEstimator.TimestampedVisionUpdate update = new VisionPoseEstimator.TimestampedVisionUpdate(
//                poseEstimate.timestampSeconds, pose, distStdDevs
//        );
//        LiveGraph.add("Vision/ReefAlignPose_" + name, poseEstimate.pose);
//
//        //if(state.gamePeriod != RobotState.GamePeriod.AUTO || (Math.abs(state.pigeonAngularVelocity) < 720 && state.hasPiece)) {
//        //if(state.gamePeriod != RobotState.GamePeriod.AUTO && Math.abs(state.pigeonAngularVelocity) < 720) {
//        if(Math.abs(state.pigeonAngularVelocity) < 720) {
//            if (!reject) state.visionPoseEstimator.addVisionData(List.of(update));
//        }
//
//        return poseEstimate.pose;
//    }
//
//    @Override
//    public void resetGyro(Commands commands, RobotState state) {
//        LimelightHelpers.SetIMUMode(name, 1);
//        if(getAlliance() == DriverStation.Alliance.Blue) {
//            LimelightHelpers.SetRobotOrientation(name, 180, 0, 0, 0, 0, 0);
//        } else {
//            LimelightHelpers.SetRobotOrientation(name, 0, 0, 0, 0, 0, 0);
//        }
//        LimelightHelpers.SetIMUMode(name, 2);
//    }
//
//    @Override
//    public void setUseGyro(RobotState state) {
//        LimelightHelpers.SetIMUMode(name, 2);
//    }
//
//    @Override
//    public void updateReefAlign(RobotState state) {
//        updateSide();
//        LiveGraph.add("Vision/Autoalign/reefAligning", state.reefAligning);
//        LiveGraph.add("Vision/Autoalign/reefAlignId", state.reefAlignId);
//        if(!state.reefAligning) { // if haven't locked on to tag, find the closest one to reefa lign to
//            int closestTag = -1;
//            double closestTagDist = Double.MAX_VALUE;
//            Pose2d robotPose = state.visionPoseEstimator.getLatestPose();
//            for(int id: reefTags) {
//                if(layout.getTagPose(id).isPresent()) {
//                    Pose2d tagPose = layout.getTagPose(id).get().toPose2d();
//                    double dist = (tagPose.getX() - robotPose.getX()) * (tagPose.getX() - robotPose.getX()) + (tagPose.getY() - robotPose.getY()) * (tagPose.getY() - robotPose.getY());
//                    if(dist < closestTagDist) {
//                        closestTag = id;
//                        closestTagDist = dist;
//                    }
//                }
//            }
//            state.reefAlignId = closestTag;
//            if(blueReefTags.containsKey(closestTag)) {
//                state.reefAlignTargetTheta = blueReefTags.get(closestTag);
//                if (layout.getTagPose(closestTag).isPresent())
//                    state.reefAlignTargetPose = layout.getTagPose(closestTag).get().toPose2d();
//            }
//        }
//
//        state.reefAlignHasTarget = false;
//        int wantedId = (int) LimelightHelpers.getFiducialID(name);
//        if(blueReefTags.containsKey(wantedId) && wantedId == state.reefAlignId) { // now publish the limelight tx ty to robotstae ONLY if the id is same as wanted
//            state.reefAlignTargetTheta = blueReefTags.get(wantedId);
//            state.reefAlignTx = LimelightHelpers.getTXNC(name);
//            state.reefAlignTy = LimelightHelpers.getTYNC(name);
//            state.reefAlignTa = LimelightHelpers.getTA(name);
//            LimelightHelpers.SetIMUMode(name, 1);
//            state.reefAlignCameraToTag = LimelightHelpers.getCameraPose3d_TargetSpace(name);
//            state.botToTag = LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace(name));
//            LiveGraph.add("Vision/3dTags/YawToTarget", LimelightHelpers.getBotPose_TargetSpace(name)[4]);
//            state.yawToTag = LimelightHelpers.getBotPose_TargetSpace(name)[4];
//            double[] botTargetSpace = LimelightHelpers.getBotPose_TargetSpace(name);
//            double[] camTargetSpace = LimelightHelpers.getCameraPose_TargetSpace(name);
//            if (camTargetSpace.length > 0) {
//                state.camTargetSpace = camTargetSpace;
//            }
//            if (botTargetSpace.length > 0) {
//                LiveGraph.add("Vision/3dTags/YawToTarget", camTargetSpace[4]);
//                state.yawToTag = camTargetSpace[4];
//            }
//            LimelightHelpers.SetIMUMode(name, 2);
//            state.reefAlignHasTarget = true;
//
//            LiveGraph.add("Vision/reefTa", state.reefAlignTa);
//            LiveGraph.add("Vision/2dTags/Tx", state.reefAlignTx);
//            LiveGraph.add("Vision/3dTags/Pose", state.reefAlignCameraToTag.toPose2d());
//        } else {
//            state.reefAlignHasTarget = false;
//        }
//
//        Set<Integer> flip = Set.of(
//                20, 21, 22, 9, 10, 11
//        );
//
//        state.reefAlignFlip = flip.contains(state.reefAlignId);
//    }
//    private void correctGyro(double yaw) {
//        LimelightHelpers.SetIMUMode(name, 1);
//        LimelightHelpers.SetRobotOrientation(name, yaw, 0, 0, 0, 0, 0);
//        LimelightHelpers.SetIMUMode(name, 2);
//    }
//}
