package frc2025.commands;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.util.Alliance;

public class DriveCommands {
    public static Command autoTrench(RobotContainer robotContainer) {
        Pose2d startPose = RobotState.getInstance().getVisionPose();

        boolean nearBlueTrench = Math.abs(startPose.getX() - FieldConstants.blueTrenchX) < 1;
        boolean nearRedTrench = Math.abs(startPose.getX() - FieldConstants.redTrenchX) < 1;
        boolean nearTrenchY = startPose.getY() < 1.982 || startPose.getY() > 6.170;

        if (!((nearBlueTrench || nearRedTrench) && nearTrenchY)) return new InstantCommand();

        Pose2d closestStartTrench = FieldConstants.trenchPoses[0];
        Pose2d endTrench;
        int trenchID = 0;
        double minDist = closestStartTrench.getTranslation().getDistance(startPose.getTranslation());

        for (int i = 1; i < FieldConstants.trenchPoses.length; i++) {
            double curDist = FieldConstants.trenchPoses[i].getTranslation().getDistance(startPose.getTranslation());
            if (curDist < minDist) {
                minDist = curDist;
                trenchID = i;
                closestStartTrench = FieldConstants.trenchPoses[i];
            }
        }

        if (trenchID % 2 == 0) {
            endTrench = FieldConstants.trenchPoses[trenchID + 1];
        } else {
            endTrench = FieldConstants.trenchPoses[trenchID - 1];
        }

//        Logger.recordOutput("Drive/autoTrenchStart", closestStartTrench);
//        Logger.recordOutput("Drive/autoTrenchEnd", endTrench);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, closestStartTrench, endTrench);
        PathConstraints constraints = new PathConstraints(2.0, 2.0, 2*Math.PI, 2*Math.PI);
        IdealStartingState idealStartingState = new IdealStartingState(0, closestStartTrench.getRotation());
        GoalEndState goalEndState = new GoalEndState(0, endTrench.getRotation());

        PathPlannerPath p = new PathPlannerPath(
                waypoints,
                constraints,
                idealStartingState,
                goalEndState,
                false
        );

        return robotContainer.getDrive().followPathCommand(p)
                .andThen(new DriveToPoseCommand(robotContainer.getDrive(), () -> endTrench));

    }

    public static Command autoTrenchReturn(RobotContainer robotContainer, boolean isHigh) {
        Pose2d preTrenchPose = (isHigh) ? FieldConstants.redTrenchHighMid : FieldConstants.redTrenchLowMid;
        Pose2d endTrench = (isHigh) ? FieldConstants.redTrenchHighZone : FieldConstants.redTrenchLowZone;
        preTrenchPose = new Pose2d(preTrenchPose.getX() - 0.5, preTrenchPose.getY(), preTrenchPose.getRotation());

        if (!Alliance.get()) {
            preTrenchPose = FlippingUtil.flipFieldPose(preTrenchPose);
            endTrench = FlippingUtil.flipFieldPose(endTrench);
        }

        Waypoint preTrench = new Waypoint(preTrenchPose.getTranslation(), preTrenchPose.getTranslation(), preTrenchPose.getTranslation());
        Waypoint end = new Waypoint(endTrench.getTranslation(), endTrench.getTranslation(), endTrench.getTranslation());

        List<Waypoint> waypoints = List.of(preTrench, end);
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2*Math.PI, 2*Math.PI);
        IdealStartingState idealStartingState = new IdealStartingState(0, preTrenchPose.getRotation());
        GoalEndState goalEndState = new GoalEndState(0, endTrench.getRotation());

        PathPlannerPath p = new PathPlannerPath(
                waypoints,
                constraints,
                idealStartingState,
                goalEndState,
                false
        );

        if (!Alliance.get()) {
            p = p.flipPath();
        }


//        Logger.recordOutput("Drive/isHigh", isHigh);
//        Logger.recordOutput("Drive/autoTrenchPre", preTrenchPose);

        Pose2d finalPreTrenchPose = preTrenchPose;
        PathPlannerPath finalP = p;
        Pose2d finalEndTrench = endTrench;
        return new DriveToPoseCommand(robotContainer.getDrive(), () ->
                new Pose2d(RobotState.getInstance().getVisionPose().getX(),
                    finalPreTrenchPose.getY(),
                    finalPreTrenchPose.getRotation()))
                .andThen(new DriveToPoseCommand(robotContainer.getDrive(), () -> finalPreTrenchPose))
                .andThen(robotContainer.getDrive().followPathCommand(finalP))
                .andThen(new DriveToPoseCommand(robotContainer.getDrive(), () -> finalEndTrench));
    }
}
