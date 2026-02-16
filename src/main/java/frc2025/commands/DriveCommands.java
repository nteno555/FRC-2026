package frc2025.commands;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class DriveCommands {
    public static Command autoTrench(RobotContainer robotContainer) {
        Pose2d startPose = RobotState.getInstance().getVisionPose();
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

        Pose2d finalClosestStartTrench = closestStartTrench;
        Command driveTrench = new DriveToPoseCommand(robotContainer.getDrive(), () -> finalClosestStartTrench);

        Logger.recordOutput("Drive/autoTrenchStart", closestStartTrench);
        Logger.recordOutput("Drive/autoTrenchEnd", endTrench);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(finalClosestStartTrench, endTrench);
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

        return driveTrench.andThen(
                new FollowPathCommand(robotContainer.getDrive(), () -> p, () -> false, false)
        );
    }
}
