package frc2025.commands;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.subsystems.Superstructure;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.util.AlignPathBuilder;
import frc2025.util.Alliance;

public class AutoScoreCommands {
    private static PathPlannerPath teleopScorePath;
    private static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private static Map<Integer, Integer> blueReefTags = Map.of(
            17, 0,
            18, 2,
            19, 4,
            20, 6,
            21, 8,
            22, 10);
    private static Map<Integer, Integer> redReefTags = Map.of(
            10, 8,
            9, 10,
            8, 0,
            7, 2,
            6, 4,
            11, 6);

    public static Command teleopAutoScoreCommand(
            DriveSubsystem driveSubsystem,
            Superstructure superstructure,
            Supplier<Integer> wantedIdSupplier,
            Supplier<Integer> level,
            Supplier<Boolean> wantScore
    ) {
        return new InstantCommand(() -> {
            Logger.recordOutput("Drive/reefId", wantedIdSupplier.get());
            teleopScorePath = AlignPathBuilder.buildFullFieldPath(RobotState.getInstance().getVisionPose(), wantedIdSupplier.get(), false, new Pose2d(), level.get() == 2);
        }).andThen(autoScoreCommand(
                driveSubsystem,
                superstructure,
                () -> teleopScorePath,
                () -> new Pose2d(
                    teleopScorePath.getPathPoses().get(teleopScorePath.numPoints()-1).getTranslation(),
                    teleopScorePath.getGoalEndState().rotation()
                ),
                level,
                wantScore));
    }

    public static Command teleopAutoScoreCommandClosest (
            DriveSubsystem drive,
            Superstructure superstructure,
            Supplier<Integer> level,
            Supplier<Boolean> wantLeft
    ) {
        return new InstantCommand(() -> {
            Map<Integer, Integer> tags;
            if(Alliance.get()) tags = redReefTags;
            else tags = blueReefTags;

            int closest = 0;
            double minDist = 999999;
            for(int tag: tags.keySet()) {
                Pose2d pose = layout.getTagPose(tag).get().toPose2d();
                double dist = RobotState.getInstance().getVisionPose().getTranslation().getDistance(pose.getTranslation());
                if(dist < minDist) {
                    closest = tags.get(tag) + (wantLeft.get() ? 1 : 0);
                    minDist = dist;
                }
            }

            Logger.recordOutput("Drive/scoreCoralClosest", closest);

            teleopScorePath = AlignPathBuilder.buildFullFieldPath(RobotState.getInstance().getVisionPose(), closest, false, new Pose2d(), level.get() == 2);
        }).andThen(new ConditionalCommand(autoScoreCommand(
                        drive,
                        superstructure,
                        () -> teleopScorePath,
                        () -> new Pose2d(
                                teleopScorePath.getPathPoses().get(teleopScorePath.numPoints()-1).getTranslation(),
                                teleopScorePath.getGoalEndState().rotation()
                        ),
                        level,
                        () -> true), 
                        autoScoreL2Command(
                        drive,
                        superstructure,
                        () -> teleopScorePath,
                        () -> new Pose2d(
                                teleopScorePath.getPathPoses().get(teleopScorePath.numPoints()-1).getTranslation(),
                                teleopScorePath.getGoalEndState().rotation()
                        ),
                        () -> true),
                        () -> level.get() != 2)
        );
    }

    public static Command autoScoreL2Command (
        DriveSubsystem driveSubsystem,
        Superstructure superstructure,
        Supplier<PathPlannerPath> pathSupplier,
        Supplier<Pose2d> endPose,
        Supplier<Boolean> wantScore
        ) {
        Command intakeCommand = new ConditionalCommand(
                IntakeCommands.intakeFunnelCommandNoStow(superstructure),
                new InstantCommand(),
                () -> !superstructure.hasPiece());
                
        Command extendElevatorArmCommand = new WaitUntilCommand(() -> superstructure.hasPiece())
                .andThen(superstructure.setWantedStateCommand(() -> Superstructure.State.L2_CORAL_PRE));

        Command followPathCommand = new FollowPathCommand(driveSubsystem, pathSupplier, () -> false, 0.02);
        return followPathCommand
                        .alongWith(
                                intakeCommand
                                        .andThen(new WaitUntilCommand(() -> RobotState.getInstance().getSpeed() < 2))
                                        .andThen(extendElevatorArmCommand)
                        )
                        .andThen(new WaitCommand(0.2))
                        .andThen(superstructure.setWantedStateCommand(Superstructure.State.EJECT_CORAL));
    }


    public static Command finalAlignDriveCommand(
            DriveSubsystem driveSubsystem,
            Supplier<Pose2d> endPose,
            Supplier<Double> speed
    ) {
        return driveSubsystem.driveRobotRelative(() -> {
            Pose2d pose = RobotState.getInstance().getVisionPose();
            Pose2d realEndPose = new Pose2d(
                    endPose.get().getTranslation().plus(new Translation2d(0.5, 0).rotateBy(endPose.get().getRotation())),
                    endPose.get().getRotation()
            );
            Logger.recordOutput("Drive/alignEndReal", realEndPose);
            Translation2d translation = realEndPose.getTranslation().minus(pose.getTranslation()).times(1);
            translation = translation.div(translation.getNorm());

            //double pidSpeed = 0.5 * translation.getNorm();

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX() * speed.get(),
                    translation.getY() * speed.get(),
                    5 * endPose.get().getRotation().minus(pose.getRotation()).getRadians(),
                    pose.getRotation()
            );
        });
    }

    public static Command autoScoreCommand(
            DriveSubsystem driveSubsystem,
            Superstructure superstructure,
            Supplier<PathPlannerPath> pathSupplier,
            Supplier<Pose2d> endPose,
            Supplier<Integer> level,
            Supplier<Boolean> wantScore
    ) {
        Command intakeCommand = new ConditionalCommand(
                IntakeCommands.intakeFunnelCommandNoStow(superstructure),
                new InstantCommand(),
                () -> !superstructure.hasPiece());

        Command extendElevatorArmCommand = new WaitUntilCommand(() -> superstructure.hasPiece())
                .andThen(superstructure.setWantedStateCommand(() -> convertLevelToState(level)));

        Command finalAlignThenScoreCommand =
                finalAlignDriveCommand(
                        driveSubsystem,
                        endPose,
                        () -> 0.5)
                        .raceWith(
                                new WaitUntilCommand(() -> RobotState.getInstance().getReefDepth() <  (0.55))
                                        .andThen(superstructure.setWantedStateCommand(() -> Superstructure.State.EJECT_CORAL)
                                                .alongWith(new WaitCommand(0.1)))
                        );

        Command followPathCommand =
                new FollowPathCommand(driveSubsystem, pathSupplier, () -> false, 0.15);


        return followPathCommand
                        .alongWith(
                                intakeCommand
                                        .andThen(new WaitUntilCommand(() -> RobotState.getInstance().getSpeed() < 2))
                                        .andThen(extendElevatorArmCommand)
                        )
                        .andThen(finalAlignThenScoreCommand);
    }

    public static Command autoScoreNetCommand(DriveSubsystem drive, Superstructure superstructure) {
        return
                (drive.driveFieldRelative(() -> new ChassisSpeeds(0, 0, 0))
                        .withTimeout(0.2))
                .andThen(superstructure.setWantedStateCommand(Superstructure.State.NET_SCORE_PRE))
                .andThen(drive.driveFieldRelativeVision(() -> new ChassisSpeeds(Alliance.get() ? -0.5 : 0.5, 0, 0))
                        .until(() -> RobotState.getInstance().getNetDepth() < -0.1))
                .andThen(superstructure.setWantedStateCommand(Superstructure.State.NET_SCORE_EJECT))
                .andThen(new WaitCommand(0.5))
                .andThen(
                        drive.applyChassisSpeedsCommand(() -> new ChassisSpeeds(0, 0, 0))
                                .withTimeout(0.01)
                );
    }

    public static Command autoScoreProcessorCommand(DriveSubsystem drive, Superstructure superstructure) {
        return superstructure.setWantedStateCommand(Superstructure.State.PROCESSOR_SCORE_PRE)
                .andThen(new DriveToPoseCommand(drive, () -> Alliance.get() ? FieldConstants.redProcessorPos : FieldConstants.blueProcessorPos))
                .andThen(superstructure.setWantedStateCommand(Superstructure.State.PROCESSOR_SCORE_EJECT));
    }

    private static Superstructure.State convertLevelToState(Supplier<Integer> level) {
        return switch (level.get()) {
            case 4 -> Superstructure.State.L4_CORAL_PRE;
            case 3 -> Superstructure.State.L3_CORAL_PRE;
            case 2 -> Superstructure.State.L2_CORAL_PRE;
            default -> Superstructure.State.L4_CORAL_PRE;
        };
    }
}
