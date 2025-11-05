package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2025.commands.AutoScoreCommands;
import frc2025.commands.FollowPathCommand;
import frc2025.commands.IntakeCommands;
import frc2025.subsystems.Superstructure;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.util.Alliance;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.function.Supplier;

public class Autos {
    private static Pose2d LOW_START = new Pose2d(6.959, 2.116, Rotation2d.fromDegrees(180));
    private static Pose2d MID_START = new Pose2d(7.100, 4.068, Rotation2d.fromDegrees(180));
    private DriveSubsystem drive;
    private Superstructure superstructure;

    public Autos(DriveSubsystem drive, Superstructure superstructure) {
        this.drive = drive;
        this.superstructure = superstructure;
    }

    private Command autoScore(int id, int level) {
        return AutoScoreCommands.teleopAutoScoreCommand(drive, superstructure, () -> id, () -> level, () -> true)
                .withTimeout(5);
        //.andThen(new WaitCommand(0.5));
        //return AutoScoreCommands.teleopAutoScoreCommandClosest(drive, superstructure, () -> level, () -> id % 2 == 1)
        //        .andThen(new WaitCommand(0.2));
    }

    private Command intake(int id, boolean lowStation) {
        String pathName = "";
        if (id == 10) pathName += "S50";
        if (id == 11) pathName += "S51";
        if (id == 0) pathName += "S60";
        if (id == 1) pathName += "S61";
        if (id == 2) pathName += "S00";
        if (id == 3) pathName += "S01";

        //if(lowStation) pathName += "-C2";
        //else pathName += "-C1";
        pathName += "-C2";

        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }
        if (!lowStation) path = path.mirrorPath();

        PathPlannerPath finalPath = path;
        return IntakeCommands.intakePath(drive, superstructure, () -> finalPath)
                .andThen(new WaitCommand(0.2));
    }

    private Pose2d mirrorPose(Pose2d pose) {
        Translation2d translation = pose.getTranslation();
        return new Pose2d(translation.getX(), FlippingUtil.fieldSizeY - translation.getY(), pose.getRotation().times(-1));
    }

    private Supplier<Pose2d> startPoseSupplier(Pose2d pose) {
        return () -> {
            if (Alliance.get()) {
                return FlippingUtil.flipFieldPose(pose);
            }
            return pose;
        };
    }

    public Command Low4Piece() {
        Command auto =
                drive.resetOdometryCommand(startPoseSupplier(LOW_START))
                        .andThen(autoScore(11, 4))
                        .andThen(intake(11, true))
                        .andThen(autoScore(0, 4))
                        .andThen(intake(0, true))
                        .andThen(autoScore(1, 4))
                        .andThen(intake(1, true))
                        .andThen(autoScore(2, 4));
        return auto;
    }

    public Command High4Piece() {
        Command auto =
                drive.resetOdometryCommand(startPoseSupplier(mirrorPose(LOW_START)))
                        .andThen(autoScore(6, 4))
                        .andThen(intake(11, false))
                        .andThen(autoScore(5, 4))
                        .andThen(intake(0, false))
                        .andThen(autoScore(4, 4))
                        .andThen(intake(1, false))
                        .andThen(autoScore(3, 4));
        return auto;
    }

    public Command Mid1CoralThenAlgae() {
        PathPlannerPath p;
        try {
            p = PathPlannerPath.fromPathFile("S41-Barge");
        } catch (IOException | ParseException e) {
            System.out.println("you're trippin");
            return null;
        }

        System.out.println(p);

        //Supplier<PathPlannerPath> pathSupplier;
        //if (p != null)  pathSupplier = () -> p;
        //else return new FollowPathCommand(drive, () -> null, () -> Alliance.get());
        //pathSupplier = () -> p;

        Command algaePath = new FollowPathCommand(drive, () -> p, () -> Alliance.get(), true);

        Command auto =
                drive.resetOdometryCommand(startPoseSupplier(MID_START))
                        .andThen(autoScore(9, 4)
                                //.andThen(IntakeCommands.intakeAlgaePath(drive, superstructure, () -> 2, () -> null))
                                //.andThen(IntakeCommands.intakeAlgaePath(drive, superstructure, () -> 2, () -> null))
                                .andThen(IntakeCommands.intakeAlgaePathClosest(drive, superstructure))
                                .andThen(algaePath)
                                .andThen(AutoScoreCommands.autoScoreNetCommand(drive, superstructure))
                                .andThen(superstructure.setWantedStateCommand(Superstructure.State.STOW)));
        return auto;
    }

    public Command Taxi() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(new Pose2d(0, 0, new Rotation2d(Math.PI))))
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                        .withTimeout(2.0))
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }
}
