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
import frc2025.config.ShooterTreeMap;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.util.Alliance;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

public class Autos {
    private DriveSubsystem drive;
    private static Pose2d HIGH_START = new Pose2d(3.516, 5.941, Rotation2d.fromDegrees(0));
    private static Pose2d MID_START = new Pose2d(3.445, 4.092, Rotation2d.fromDegrees(0));
    private static Pose2d LOW_START = new Pose2d(3.516, 2.112, Rotation2d.fromDegrees(0));

    private RobotContainer robotContainer;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrive();
        RobotState.getInstance().setShuttling(false);
    }

    /* Path Commands */

    private Command intakePath(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;
        return new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true)
                .raceWith(robotContainer.getIntake().setIntaking(() -> true));
                //.andThen(new WaitCommand(0.2));
    }

    private Command intakePathResetOdometry(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;
        Pose2d startPose = new Pose2d(
                finalPath.getStartingDifferentialPose().getTranslation(),
                finalPath.getIdealStartingState().rotation()
        );
        return  drive.resetOdometryCommand(() -> startPose)
                .andThen(new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true))
                .raceWith(robotContainer.getIntake().setIntaking(() -> true));
    }

    private Command finalAutoAim() {
        return AutoAimCommands.driveAutoAim(
                robotContainer,
                () -> RobotState.getInstance().getTargetPose(),
                () -> new ChassisSpeeds(0,0,0)
        );
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

    public Command buildAuto(List<String> paths, boolean climb) {
        Command auto = AutoAimCommands.turretAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(), () -> null)
                .alongWith(AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose()))
                .alongWith(AutoAimCommands.hoodAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose()))
                .alongWith(AutoAimCommands.autoIndex(robotContainer));

        Command pathCommands = intakePathResetOdometry(paths.get(0));

        for(int i = 1; i < paths.size(); i++) {
            pathCommands = pathCommands.andThen(intakePath(paths.get(i)));
        }

        return auto.andThen(pathCommands);
    }


    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }

    public Command HighDepotStationClimb = buildAuto(List.of("High-Depot", "Depot-Station", "Station-Climb"), true);
    public Command MidStationDepotClimb = buildAuto(List.of("Mid-Station", "Station-Depot", "Depot-Climb"), true);
    public Command LowStationDepotClimb = buildAuto(List.of("Low-Station", "Station-Depot", "Depot-Climb"), true);
    public Command HighDepotClimb = buildAuto(List.of("High-Depot", "Depot-Climb"), true);
    public Command LowStationClimb = buildAuto(List.of("Low-Station", "Station-Climb"), true);
    public Command MidClimb = buildAuto(List.of("Mid-Climb"), true);
    public Command LowStationNeutralClimb = buildAuto(List.of("Low-Station", "Station-Neutral", "Neutral-Climb (Low)"), true);
}