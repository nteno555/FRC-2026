package frc2025.util;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc2025.config.FieldConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static edu.wpi.first.wpilibj.DriverStation.getAlliance;

public class AlignPathBuilder {
    public static int targetAlignTag = -1;
    // Between 1-12:
    public static int wantedAlignId = 0;
    public static int wantedIntakeId = 0;
    private static HashMap<String, Integer> nameToId = new HashMap<>();
    //private static HashMap<String, Integer> nameToIdRed = new HashMap<>();
    private static HashMap<Integer, Integer> idToTagRed = new HashMap<>();
    private static ArrayList<Pose2d> redCorners = new ArrayList<>();
    private static ArrayList<Pose2d> blueCorners = new ArrayList<>();

    private static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static void initialize()  {
        nameToId.put("S60", 0);
        nameToId.put("S61", 1);
        nameToId.put("S10", 2);
        nameToId.put("S11", 3);
        nameToId.put("S20", 4);
        nameToId.put("S21", 5);
        nameToId.put("S30", 6);
        nameToId.put("S31", 7);
        nameToId.put("S40", 8);
        nameToId.put("S41", 9);
        nameToId.put("S50", 10);
        nameToId.put("S51", 11);

        idToTagRed.put(0, 8);
        idToTagRed.put(1, 8);
        idToTagRed.put(2, 7);
        idToTagRed.put(3, 7);
        idToTagRed.put(4, 6);
        idToTagRed.put(5, 6);
        idToTagRed.put(6, 11);
        idToTagRed.put(7, 11);
        idToTagRed.put(8, 10);
        idToTagRed.put(9, 10);
        idToTagRed.put(10, 9);
        idToTagRed.put(11, 9);

        /*if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {

            TXReefMap.intakePositions.put(0, FlippingUtil.flipFieldPose(TXReefMap.intakePositions.get(0)));
            TXReefMap.intakePositions.put(1, FlippingUtil.flipFieldPose(TXReefMap.intakePositions.get(1)));
        }*/

        double distanceFromReef = -1.95;
        Pose2d blueCenter = new Pose2d(
                4.478, 4.022, Rotation2d.fromDegrees(0)
        );

        Pose2d redCenter = new Pose2d(
                4.478 + 8.617, 4.022, Rotation2d.fromDegrees(0)
        );

        for (int i = 30; i <= 330; i+= 60) {
            Translation2d trans = new Translation2d(distanceFromReef, 0);
            Transform2d transform = new Transform2d(trans.rotateBy(Rotation2d.fromDegrees(i)), Rotation2d.kZero);
            Pose2d redPoint = redCenter.transformBy(transform);
            redPoint = new Pose2d(redPoint.getTranslation(), Rotation2d.fromDegrees(i + 180));
            redCorners.add(redPoint);
            Pose2d bluePoint = blueCenter.transformBy(transform);
            bluePoint = new Pose2d(bluePoint.getTranslation(), Rotation2d.fromDegrees(i + 180));
            blueCorners.add(bluePoint);
            System.out.println("Red Point");
            System.out.println("X: " + redPoint.getTranslation().getX());
            System.out.println("Y: " + redPoint.getTranslation().getY());
            System.out.println("Rotation2D: " + redPoint.getRotation().getDegrees());
        }

    }
    public static int getNearestRemoveId(Pose2d robotPose) {
        int closestId = -1;
        double closestTagDist = Double.MAX_VALUE;
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        for(int id = 0; id < 12; id++) {
            int targetTag;
            if(!isRed) {
                targetTag = id/2 + 17;
            } else {
                targetTag = idToTagRed.get(id);
            }
            if(layout.getTagPose(targetTag).isPresent()) {
                Pose2d tagPose = layout.getTagPose(targetTag).get().toPose2d();
                double dist = calculateDistance(tagPose, robotPose);
                if(dist < closestTagDist) {
                    System.out.println(targetTag);
                    closestId = id;
                    closestTagDist = dist;
                }
            }
        }
        return closestId;
    }
    private static int chooseNearestScoringLocation(Pose2d robotPose, ChassisSpeeds currentChassisSpeeds, double reactionTimeSeconds) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                currentChassisSpeeds,
                robotPose.getRotation()
        );
        Pose2d futurePose = robotPose.plus(new Transform2d(
                        fieldSpeeds.vxMetersPerSecond * reactionTimeSeconds,
                        fieldSpeeds.vyMetersPerSecond * reactionTimeSeconds,
                        new Rotation2d(fieldSpeeds.omegaRadiansPerSecond * reactionTimeSeconds)
                )
        );

        List<Integer> reefTags = getAlliance().get().equals(DriverStation.Alliance.Red) ? List.of(6, 7, 8, 9, 10, 11) : List.of(17, 18, 19, 20, 21, 22);

        int closestTag = -1;
        double closestTagDist = Double.MAX_VALUE;
        for(int id: reefTags) {
            if(layout.getTagPose(id).isPresent()) {
                Pose2d tagPose = layout.getTagPose(id).get().toPose2d();
                double dist = (tagPose.getX() - futurePose.getX()) * (tagPose.getX() - futurePose.getX()) + (tagPose.getY() - futurePose.getY()) * (tagPose.getY() - futurePose.getY());
                if(dist < closestTagDist) {
                    closestTag = id;
                    closestTagDist = dist;
                }
            }
        }
        targetAlignTag = closestTag;

        return closestTag;
    }
    private static int chooseNearestScoringLocation(Pose2d robotPose, ChassisSpeeds currentChassisSpeeds) {
        return chooseNearestScoringLocation(robotPose, currentChassisSpeeds, 0.2);
    }

    public static Pose2d getAlgaeEndPose(int targetId) {

        if (DriverStation.getAlliance().isEmpty()) return new Pose2d();//new ArrayList<>(List.of(robotPose, robotPose));
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        int tagIdOffset = isRed ? 6 : 17;

        int targetTag;
		/*int targetTag = targetID/2 + tagIdOffset;
		targetAlignTag = targetTag;
		boolean wantLeft = (targetID % 2) == 1;*/
        if(!isRed) {
            targetTag = targetId/2 + tagIdOffset;
        } else {
            targetTag = idToTagRed.get(targetId);
        }
        // find nearest reef point.
        Pose2d targetTagPose = layout.getTagPose(targetTag).get().toPose2d();



        Pose2d alignEndPose;
        double lateralOffset, yOffset;
        lateralOffset = 0;
        yOffset = 0.45;

        Translation2d tagToEndPoseTranslation = (new Translation2d(yOffset, lateralOffset)).rotateBy(targetTagPose.getRotation());
        alignEndPose = new Pose2d(targetTagPose.getTranslation().plus(tagToEndPoseTranslation), targetTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        return alignEndPose;
    }
    public static ArrayList<Pose2d> createPath(Pose2d robotPose, ChassisSpeeds currentChassisSpeeds, double reactionTimeSeconds, boolean wantLeft, boolean wantMiddle) {
        if (getAlliance().isEmpty()) return new ArrayList<>(List.of(robotPose, robotPose));
        List<Integer> reefTags = getAlliance().get().equals(DriverStation.Alliance.Red) ? List.of(6, 7, 8, 9, 10, 11) : List.of(17, 18, 19, 20, 21, 22);
        // find nearest reef point.
        int closestTag = chooseNearestScoringLocation(robotPose, currentChassisSpeeds, reactionTimeSeconds);
        if (closestTag == -1) return new ArrayList<>(List.of(robotPose, robotPose));
        Pose2d targetTagPose = layout.getTagPose(closestTag).get().toPose2d();

        double lateralOffset;
        if(wantMiddle) lateralOffset = 0;
        else lateralOffset = wantLeft ? 0.2 : -0.2;

        Translation2d tagToEndPoseTranslation = (new Translation2d(0.75, lateralOffset)).rotateBy(targetTagPose.getRotation());
        Pose2d alignEndPose = new Pose2d(targetTagPose.getTranslation().plus(tagToEndPoseTranslation), targetTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

        return new ArrayList<>(List.of(robotPose, /*prealignPose,*/ alignEndPose));
    }
    public static ArrayList<Pose2d> createReefAlignPath(Pose2d robotPose, boolean wantLeft) { return createPath(robotPose, new ChassisSpeeds(), 0, wantLeft, false); }
    public static ArrayList<Pose2d> createAlgaeAlignPath(Pose2d robotPose) { return createPath(robotPose, new ChassisSpeeds(), 0, false, true); }
    public static Pose2d getEndPoseAuto(String name) {
        System.out.println("END POSE");
        System.out.println(name);
        return getEndPose(nameToId.get(name));
    }
    public static Pose2d getEndPoseAuto(int tagId, boolean wantLeft) {
		/*AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

		Pose2d targetTagPose = layout.getTagPose(tagId).get().toPose2d();

		double lateralOffset = !wantLeft ? -0.12 : 0.12;
		Translation2d tagToEndPoseTranslation = (new Translation2d(0.480, lateralOffset)).rotateBy(targetTagPose.getRotation());
		Pose2d alignEndPose = new Pose2d(targetTagPose.getTranslation().plus(tagToEndPoseTranslation), targetTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
		LiveGraph.add("Drive/alignEndPose", alignEndPose);
		return alignEndPose;*/
        //return getEndPose((tagId - 17) * 2 + (wantLeft ? 0 : 1));
        //return getEndPose()
        return null;
    }
    public static Pose2d getEndPoseAuto(Pose2d endPose, boolean wantLeft) {
        double minDist = Double.MAX_VALUE;
        int minTagId = -1;
        for(var tag: layout.getTags()) {
            double d = endPose.minus(tag.pose.toPose2d()).getTranslation().getNorm();
            if(d < minDist) {
                minDist = d;
                minTagId = tag.ID;
            }
        }
        return getEndPoseAuto(minTagId, wantLeft);
    }
    public static Pose2d getEndPose(int targetID) {
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        int targetTag;
        if(!isRed) {
            int tagIdOffset = isRed ? 6 : 17;
            targetTag = targetID / 2 + tagIdOffset;
        } else {
            targetTag = idToTagRed.get(targetID);
        }

        targetAlignTag = targetTag;
        boolean wantLeft = (targetID % 2) == 1;
        // find nearest reef point.
        Pose2d targetTagPose = layout.getTagPose(targetTag).get().toPose2d();

        Pose2d alignEndPose;
        double lateralOffset, yOffset;
        if(!isRed) {
            lateralOffset = FieldConstants.xScoreOffsetsBlue.get(targetID);//wantLeft ? -0.15 : 0.15;
            yOffset = FieldConstants.yScoreOffsetsBlue.get(targetID);
        } else {
            lateralOffset = FieldConstants.xScoreOffsetsRed.get(targetID);//wantLeft ? -0.15 : 0.15;
            yOffset = FieldConstants.yScoreOffsetsRed.get(targetID);
        }

        Translation2d tagToEndPoseTranslation = (new Translation2d(yOffset, lateralOffset)).rotateBy(targetTagPose.getRotation());
        alignEndPose = new Pose2d(targetTagPose.getTranslation().plus(tagToEndPoseTranslation), targetTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        return alignEndPose;
    }
    public static PathPlannerPath buildFullFieldPath(Pose2d robotPose, int targetID, boolean wantAlgae, Pose2d currentPosition, boolean l2) {
        PathConstraints constraints;
        // Constraints:
        if(wantAlgae) {
              constraints = new PathConstraints(
                    2, 2, 2*Math.PI, 2*Math.PI
            );
        }
        else if (l2){
            constraints = new PathConstraints(1.67, 1.67, 1.67*Math.PI, 1.67*Math.PI);
        }
        else if(RobotState.isAutonomous()) {
             constraints = new PathConstraints(
                    4, 3, 3*Math.PI, 3*Math.PI
            );
        } else {
            constraints = new PathConstraints(
                    4, 3, 3*Math.PI, 3*Math.PI
            );
        }



        // pick the nearest (AVAILABLE) corner point. that means a point that has a "view" of the robot. (+- 120 deg) btw
        // robot and corner fromthe POV of the tag.

        // same principle to figure out if we are going to hit:
        // see if theres a sightline +- 90 deg from the scoring location

        // Corner Points:
        if (wantAlgae) {
            targetID = getNearestRemoveId(robotPose);
        }
        if (DriverStation.getAlliance().isEmpty()) return null;//new ArrayList<>(List.of(robotPose, robotPose));
        boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        int targetTag;

        if(!isRed) {
            int tagIdOffset = isRed ? 6 : 17;
            targetTag = targetID / 2 + tagIdOffset;
        } else {
            targetTag = idToTagRed.get(targetID);
        }

        Logger.recordOutput("Drive/alignPathBuilderTagId", targetTag);

        targetAlignTag = targetTag;
        boolean wantLeft = (targetID % 2) == 1;
        // find nearest reef point.
        Pose2d targetTagPose = layout.getTagPose(targetTag).get().toPose2d();
        Logger.recordOutput("Drive/alignPathBuilderTagPose", targetTagPose);



        Pose2d alignEndPose;
        double lateralOffset, yOffset;
        if (!wantAlgae) {
            if(!isRed) {
                lateralOffset = FieldConstants.xScoreOffsetsBlue.get(targetID);//wantLeft ? -0.15 : 0.15;
                if(!l2) yOffset = FieldConstants.yScoreOffsetsBlue.get(targetID);
                else yOffset = FieldConstants.yScoreOffsetsBlueL2.get(targetID);
            } else {
                lateralOffset = FieldConstants.xScoreOffsetsRed.get(targetID);//wantLeft ? -0.15 : 0.15;
                if(!l2) yOffset = FieldConstants.yScoreOffsetsRed.get(targetID);
                else yOffset = FieldConstants.yScoreOffsetsRedL2.get(targetID);
            }
        } else {
            lateralOffset = 0;
            yOffset = 1;
        }

        Translation2d tagToEndPoseTranslation = (new Translation2d(yOffset, lateralOffset)).rotateBy(targetTagPose.getRotation());
        alignEndPose = new Pose2d(targetTagPose.getTranslation().plus(tagToEndPoseTranslation), targetTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
        //Pose2d alignEndReference = new Pose2d(alignEndPose.getTranslation(), alignEndPose.getRotation().rotateBy(Rotation2d.k180deg));


        //if (checkSightline(alignEndPose, robotPose, Math.PI/2 + Math.toRadians(15)) || calculateDistance(alignEndPose, robotPose) < 1.0) {
            Pose2d robotWaypoint = new Pose2d(
                    robotPose.getTranslation(),
                    Rotation2d.fromRadians(Math.atan2(alignEndPose.getY() - robotPose.getY(), alignEndPose.getX() - robotPose.getX()))
            );
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(robotWaypoint, alignEndPose);
            IdealStartingState start = new IdealStartingState(0, robotPose.getRotation());

            ArrayList<ConstraintsZone> zones = new ArrayList<>();
//			zones.add(
//					new ConstraintsZone(
//							0.9,
//							1.0,
//							new PathConstraints(
//									0.5, 0.5, 3 * Math.PI, 3 * Math.PI
//							)
//					)
//			);
            PathPlannerPath p = new PathPlannerPath(
                    waypoints,
                    new ArrayList<RotationTarget>(),
                    new ArrayList<PointTowardsZone>(),
                    zones,
                    new ArrayList<EventMarker>(),
                    constraints,
                    start,
                    new GoalEndState(0, alignEndPose.getRotation()),
                    false
            );
            Logger.recordOutput("Drive/alignPathBuilderPathEnd", p.getPathPoses().get(p.getPathPoses().size()-1));
            p.preventFlipping = true;
            PathPlannerLogging.logActivePath(p);
            int i = 0;
            for (Pose2d pose : p.getPathPoses()) {
                i++;
            }

            return p;
        //}

//        /ArrayList<Pose2d> cornerPoints;
//
//        if (isRed) {
//            cornerPoints = redCorners;
//        } else {
//            cornerPoints = blueCorners;
//        }
//
//        // Find the points that have a sightline and choose the one that is nearest to the endpoint.
//        ArrayList<Pose2d> pathPoints = new ArrayList<>(List.of(robotPose));
//        Pose2d lastPathPoint = robotPose;
//
//        int iterationCount = 0;
//
//        while (true) {
//
//            Pose2d nextPose = new Pose2d(1000000, 1000000, new Rotation2d());
//            int j = 0;
//            for (Pose2d corner : cornerPoints) {
//                Pose2d rotatedCorner = new Pose2d(corner.getTranslation(), corner.getRotation().rotateBy(Rotation2d.k180deg));
//                if (!pathPoints.contains(rotatedCorner) && !pathPoints.contains(corner)) {
//                    if (checkSightline(rotatedCorner, lastPathPoint, Math.PI*2./3. + Math.toRadians(20)) ||
//                            calculateDistance(rotatedCorner, lastPathPoint) < 2.5) {
//                        j += 1;
//                        double distanceCurrent = corner.minus(targetTagPose).getTranslation().getNorm();
//                        double distanceLast = nextPose.minus(targetTagPose).getTranslation().getNorm();
//                        if (Math.abs(distanceLast - distanceCurrent) < 0.4) {
//                            double dToBotCurrent = corner.minus(lastPathPoint).getTranslation().getNorm();
//                            double dToBotLast = nextPose.minus(lastPathPoint).getTranslation().getNorm();
//                            if (dToBotCurrent < dToBotLast) {
//                                nextPose = corner;
//                            }
//                        } else {
//                            if (distanceCurrent <= distanceLast) {
//                                nextPose = corner;
//                            }
//                        }
//                    }
//                }
//            }
//
//            Pose2d cornerPathPoint = new Pose2d(nextPose.getTranslation(), nextPose.getRotation().rotateBy(Rotation2d.k180deg));
//            pathPoints.add(cornerPathPoint);
//            lastPathPoint = cornerPathPoint;
//
//            if (calculateDistance(alignEndPose, lastPathPoint) < 2.0) {
//                pathPoints.add(alignEndPose);
//                break;
//            }
//
//            if (iterationCount >= 12) {
//                return null;
//            }
//
//            iterationCount += 1;
//        }
//
//        ArrayList<Pose2d> realpathPoints = new ArrayList<>();
//        List<RotationTarget> rotationTargets = new ArrayList<>();
//
//        int i = 0;
//        for (Pose2d pathPoint : pathPoints) {
//            if (i < pathPoints.size() - 1) {
//                Rotation2d tDirection = Rotation2d.fromRadians(Math.atan2(pathPoints.get(i+1).getY() - pathPoint.getY(), pathPoints.get(i+1).getX() - pathPoint.getX()));
//
//				/*
//				rotationTargets.add(new RotationTarget(
//
//				))
//				 */
//                realpathPoints.add(new Pose2d(
//                        pathPoint.getTranslation(),
//                        tDirection
//                ));
//
//            }
//            i++;
//        }
//        realpathPoints.add(pathPoints.get(pathPoints.size() - 1));
//
//        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(realpathPoints);
//        IdealStartingState start = null;//new IdealStartingState(0, robotPose.getRotation());
//        ArrayList<ConstraintsZone> zones = new ArrayList<>();
////		zones.add(
////				new ConstraintsZone(
////						realpathPoints.size() - 1.0,
////						realpathPoints.size(),
////						new PathConstraints(
////								0.5, 0.5, 1 * Math.PI, 1 * Math.PI
////						)
////				)
////		);
//        PathPlannerPath p = new PathPlannerPath(
//                waypoints,
//                new ArrayList<RotationTarget>(),
//                new ArrayList<PointTowardsZone>(),
//                zones,
//                new ArrayList<EventMarker>(),
//                constraints,
//                start,
//                new GoalEndState(0.0, alignEndPose.getRotation()),
//                false
//        );
//        //PathPlannerPath p = new PathPlannerPath(
//
//        //)
//        p.preventFlipping = true;
//        PathPlannerLogging.logActivePath(p);
//        int j = 0;
//        for (Pose2d pose : p.getPathPoses()) {
//            j++;
//        }
//
//        return p;
    }

    public static PathPlannerPath buildIntakePath(Pose2d robotPose, int wantedStation) {
        if(robotPose.getY() > 3.058 && robotPose.getY() < 4.993 && robotPose.getX() > 5.083) { // if in back back reef, add a waypoint. otherwise you can drive in a straight line

        }
        Pose2d endPose = FieldConstants.intakePositions.get(wantedStation);
        PathConstraints constraints = new PathConstraints(
                4, 3, 3*Math.PI, 3*Math.PI
        );

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(robotPose, endPose);
        IdealStartingState start = null;
        ArrayList<ConstraintsZone> zones = new ArrayList<>();

        PathPlannerPath p = new PathPlannerPath(
                waypoints,
                new ArrayList<RotationTarget>(),
                new ArrayList<PointTowardsZone>(),
                zones,
                new ArrayList<EventMarker>(),
                constraints,
                start,
                new GoalEndState(0.0, endPose.getRotation()),
                false
        );
        //PathPlannerPath p = new PathPlannerPath(

        //)
        p.preventFlipping = true;
        PathPlannerLogging.logActivePath(p);
        return p;
    }

    private static double minAngle(Pose2d a, Pose2d b) {
        // a is the point who has the perspective
        if (a == null || b == null) {
            return 0;
        }
        Translation2d tOffset = a.getTranslation().minus(b.getTranslation());

        double alpha = Math.atan2(tOffset.getY(), tOffset.getX());
        return ((alpha - a.getRotation().getRadians()) + 5*Math.PI) % (2*Math.PI) - Math.PI;
    }

    private static boolean checkSightline(Pose2d a, Pose2d b, double maxAngleDiff) {
        return Math.abs(minAngle(a, b)) < maxAngleDiff;
    }

    private static double calculateDistance(Pose2d a, Pose2d b) {
        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
    }
}
