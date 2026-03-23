package frc2025.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.Robot;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
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

        PathConstraints constraints = new PathConstraints(3.0, 4.0, 3*Math.PI, 4*Math.PI);
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(endVelocity, end.getRotation())
        );

        return path;
    }

    public static Command returnUnderTrenchAuto(RobotContainer robotContainer, Supplier<Pose2d> endPose, Supplier<Double> endVelocity) {
        Supplier<PathPlannerPath> preTrenchPathSupplier = () -> {
            var robotPose = RobotState.getInstance().getVisionPose();
            var xPos = robotPose.getX();
            if (xPos < FieldConstants.blueTrenchHighMid.getX() + 0.3) {
                xPos = FieldConstants.blueTrenchHighMid.getX() + 0.3;
            }
            if (xPos > FieldConstants.redTrenchHighMid.getX() - 0.3) {
                xPos = FieldConstants.redTrenchHighMid.getX() - 0.3;
            }
            var preTrenchEndPose = new Pose2d(xPos, endPose.get().getY(), endPose.get().getRotation());
            return straightPathFromPoses(robotPose, preTrenchEndPose, 0.0);
        };

        Supplier<PathPlannerPath> trenchPathSupplier = () -> {
            var robotPose = RobotState.getInstance().getVisionPose();
            var preTrenchEndPose = endPose.get();
            return straightPathFromPoses(robotPose, preTrenchEndPose, endVelocity.get());
        };

        return new FollowPathCommand(robotContainer.getDrive(), preTrenchPathSupplier, () -> false, false)
                .andThen(new FollowPathCommand(robotContainer.getDrive(), trenchPathSupplier, () -> false, true));
    }
}
