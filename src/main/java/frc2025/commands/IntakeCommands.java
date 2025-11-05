package frc2025.commands;

import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.RobotState;
import frc2025.subsystems.Superstructure;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.util.AlignPathBuilder;
import frc2025.util.Alliance;

public class IntakeCommands {
    private static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private static Map<Integer, Integer> blueReefTags = Map.of(
            17, 0,
            18, 2,
            19, 4,
            20, 6,
            21, 8,
            22, 10);
    private static Map<Integer, Integer> redReefTags = Map.of(
            6, 8,
            7, 10,
            8, 0,
            9, 2,
            10, 4,
            11, 6);

    public static Command intakeFunnelCommand(
            Superstructure superstructure
    ) {
        return
                new WaitUntilCommand(() -> superstructure.hasPiece())
                .deadlineFor(superstructure.setWantedStateCommand(Superstructure.State.HANDOFF))
                .andThen(superstructure.setWantedStateCommand(Superstructure.State.STOW));
    }

    public static Command intakeFunnelCommandNoStow(
            Superstructure superstructure
    ) {
        return
                new WaitUntilCommand(() -> superstructure.hasPiece())
                .deadlineFor(superstructure.setWantedStateCommand(Superstructure.State.HANDOFF));
    }

    public static Command intakePath(
            DriveSubsystem drive,
            Superstructure superstructure,
            Supplier<PathPlannerPath> pathSupplier
    ) {
        return new FollowPathCommand(drive, pathSupplier, () -> Alliance.get(), true)
                .deadlineFor(intakeFunnelCommand(superstructure));
    }

    public static Command intakeAlgaePath(
            DriveSubsystem drive,
            Superstructure superstructure,
            Supplier<Integer> level,
            Supplier<PathPlannerPath> pathSupplier
    ) {
        return
                new FollowPathCommand(drive, pathSupplier, () -> false, false)
                        .alongWith(superstructure.setWantedStateCommand(() -> {
                            return switch (level.get()) {
                                case 2 -> Superstructure.State.L2_ALGAE_INTAKE;
                                case 3 -> Superstructure.State.L3_ALGAE_INTAKE;
                                default -> Superstructure.State.L3_ALGAE_INTAKE;
                            };
                        }))
                        .andThen(drive.applyChassisSpeedsCommand(() -> new ChassisSpeeds(0.5, 0, 0))
                                .withTimeout(1.0));
    }

    public static Command intakeAlgaePathClosest(
            DriveSubsystem drive,
            Superstructure superstructure
    ) {
        return intakeAlgaePath(
                drive,
                superstructure,
                () -> {
                    Map<Integer, Integer> tags;
                    if(Alliance.get()) tags = redReefTags;
                    else tags = blueReefTags;

                    int closest = 0;
                    double minDist = 999999;
                    for(int tag: tags.keySet()) {
                        Pose2d pose = layout.getTagPose(tag).get().toPose2d();
                        double dist = RobotState.getInstance().getVisionPose().getTranslation().getDistance(pose.getTranslation());
                        if(dist < minDist) {
                            closest = tags.get(tag);
                            minDist = dist;
                        }
                    }

                    if(closest/2 % 2 == 0) return 2;
                    else return 3;
                },
                () -> {
                    Map<Integer, Integer> tags;
                    if (Alliance.get()) tags = redReefTags;
                    else tags = blueReefTags;

                    int closest = 0;
                    double minDist = 999999;
                    for (int tag : tags.keySet()) {
                        Pose2d pose = layout.getTagPose(tag).get().toPose2d();
                        double dist = RobotState.getInstance().getVisionPose().getTranslation().getDistance(pose.getTranslation());
                        if (dist < minDist) {
                            closest = tags.get(tag);
                            minDist = dist;
                        }
                    }

                    return AlignPathBuilder.buildFullFieldPath(RobotState.getInstance().getVisionPose(), closest, true, new Pose2d(), false);
                }
        );
    }
}
