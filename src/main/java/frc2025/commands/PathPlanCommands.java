package frc2025.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer;
import frc2025.RobotState;

import java.util.List;
import java.util.function.Supplier;

public class PathPlanCommands {
    public enum TrenchPos {
        HIGH, LOW
    }

    private static Pose2d withZeroRotation(Pose2d pose) {
        return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(0));
    }

    private static PathPlannerPath straightPathFromPoses(Pose2d start, Pose2d end, double endVelocity) {
        Pose2d startWaypoint = new Pose2d(
                start.getTranslation(),
                Rotation2d.fromRadians(Math.atan2(end.getY() - start.getY(), end.getX() - start.getX())) // direction of velocity
        );
        Pose2d endWaypoint = new Pose2d(
                end.getTranslation(),
                Rotation2d.fromRadians(Math.atan2(end.getY() - start.getY(), end.getX() - start.getX())) // direction of velocity
        );

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                startWaypoint,
                endWaypoint
        );

        PathConstraints constraints = new PathConstraints(3.5, 4, 2*Math.PI, 2*Math.PI);
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(endVelocity, end.getRotation())
        );

        return path;
    }

    public static Command returnUnderTrenchAuto(RobotContainer robotContainer, Supplier<Pose2d> preTrenchPose, Supplier<Pose2d> trenchEndPose, Supplier<Double> endVelocity) {
//        Supplier<PathPlannerPath> preTrenchPathSupplier = () -> {
//            var robotPose = RobotState.getInstance().getVisionPose();
//            var xPos = robotPose.getX();
////            if (xPos < FieldConstants.blueTrenchHighMid.getX() + 0.2) {
////                xPos = FieldConstants.blueTrenchHighMid.getX() + 0.2;
////            }
////            if (xPos > FieldConstants.redTrenchHighMid.getX() - 0.2) {
////                xPos = FieldConstants.redTrenchHighMid.getX() - 0.2;
////            }
//            var preTrenchEndPose = new Pose2d(xPos, endPose.get().getY(), endPose.get().getRotation());
//            return straightPathFromPoses(robotPose, preTrenchEndPose, 0.0);
//        };
//

//        Logger.recordOutput("Drive/preTrenchPose", preTrenchPose.get());

        Supplier<PathPlannerPath> preTrenchPathSupplier = () -> {
            var robotPose = RobotState.getInstance().getVisionPose();
            return straightPathFromPoses(robotPose, preTrenchPose.get(), 0.0);
        };

        Supplier<PathPlannerPath> trenchEndPathSupplier = () -> {
            var robotPose = RobotState.getInstance().getVisionPose();
            return straightPathFromPoses(robotPose, trenchEndPose.get(), endVelocity.get());
        };

        return new FollowPathCommand(robotContainer.getDrive(), preTrenchPathSupplier, () -> false, false).until(() -> {
            var robotPose = RobotState.getInstance().getVisionPose();
            return (Math.abs(robotPose.getY() - preTrenchPose.get().getY()) < 0.15) && (Math.abs(robotPose.getRotation().getDegrees() - preTrenchPose.get().getRotation().getDegrees()) < 10);
        }).andThen(
                new FollowPathCommand(robotContainer.getDrive(), trenchEndPathSupplier, () -> false, false)
        );
    }
}
