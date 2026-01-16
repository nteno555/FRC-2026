package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.FollowPathCommand;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.util.Alliance;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

public class Autos {
    private DriveSubsystem drive;
    private static Pose2d HIGH_START = new Pose2d(3.516, 5.941, Rotation2d.fromDegrees(180));
    private static Pose2d MID_START = new Pose2d(3.516, 4.092, Rotation2d.fromDegrees(180));
    private static Pose2d LOW_START = new Pose2d(3.516, 2.112, Rotation2d.fromDegrees(180));
    private static Pose2d HighDriveback = new Pose2d(3.000, 5.941, new Rotation2d(0));
    private static Pose2d MidDriveback = new Pose2d(3.000, 4.092, new Rotation2d(0));
    private static Pose2d LowDriveback = new Pose2d(3.000, 2.112, new Rotation2d(0));

    public static Pose2d startPose = HIGH_START;
    private RobotContainer robotContainer;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrive();
    }

    public Command autoScore(Pose2d pose) {
        Command score = AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> pose)
                .andThen(
                        new WaitUntilCommand(() -> {
                            double vx = drive.getFieldRelativeSpeeds().vxMetersPerSecond;
                            double vy = drive.getFieldRelativeSpeeds().vyMetersPerSecond;
                            double velocity = Math.hypot(vx, vy);
                            return robotContainer.shooterSubsystem.getShotReady() && velocity < SubsystemConstants.kMaxShootVelocity;
                        })
                                .andThen(SuperstructureCommands.shootCommand(robotContainer)));
        return score;
    }

    private Command intake(int startingID, int endingID) {
        /* 0: Low, 1: Mid, 2: High, 3: Depot, 4: Climb, 5: Neutral, 6: Driveback, 7: HighDriveback, 8: MidDriveback, 9: LowDriveback, 10: Station*/
        String[] positions = {"Low", "Mid", "High", "Depot", "Climb", "Neutral", "Driveback", "HighDriveback", "MidDriveback", "LowDriveback", "Station"};
        String pathName = positions[startingID] + "-" + positions[endingID];

        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;
        return new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true)
                .andThen(new WaitCommand(0.2));
    }

    public Command intakeCommand() {
        return robotContainer.getIntake().setVoltage(() -> 5.0);
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
    public Command LowShootPreload() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(LOW_START))
                .andThen(intake(0, 6))
                .andThen(autoScore(new Pose2d(3.000, 4.092, new Rotation2d(0))));
        return auto;
    }

    public Command HighShootClimb() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(intake(2, 6))
                .andThen(autoScore(HighDriveback))
                .andThen(intake(7, 4));
        return auto;

    }
    public Command HighShootIntakeClimb() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(intake(2, 6))
                .andThen(autoScore(HighDriveback))
                .andThen(intake(7, 3).alongWith(intakeCommand()))
                .andThen(intake(3, 7))
                .andThen(autoScore(HighDriveback))
                .andThen(intake(7, 4));
        return auto;
    }

    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }
}