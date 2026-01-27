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
        return drive.resetOdometryCommand(() -> startPose)
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

    /* Low Autos */
    public Command LowShootPreload() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(LOW_START))
                .andThen(
                        AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
                                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                                .alongWith(
                                        intakePath("Low-Driveback")
                                )
                );
        return auto;
    }


    /* Mid Autos */

    public Command MidDepotNeutralClimbShootWhileMoving() {
//        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
//                .andThen(
//                        AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
//                                .alongWith(AutoAimCommands.autoIndex(robotContainer))
//                                .alongWith(
//                                        intakePath("Mid-Depot")
//                                                .andThen(intakePath("Depot-Neutral"))
//                                                .andThen(intakePath("Neutral-Climb"))
//                                                .andThen(finalAutoAim())
//                                )
//                );
        Command auto = AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
                .alongWith(AutoAimCommands.hoodAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose()))
                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                .alongWith(
                        intakePathResetOdometry("Mid-Depot")
                                .andThen(intakePath("Depot-Neutral"))
                                .andThen(intakePath("Neutral-Climb"))
                                .andThen(finalAutoAim())
                );
        return auto;
    }

    public Command MidDepotShootWhileMoving() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(
                        AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
                                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                                .alongWith(
                                        intakePath("Mid-Depot")
                                                .andThen(intakePath("Depot-Mid"))
                                                .andThen(finalAutoAim())
                                )
                );
        return auto;
    }

    public Command MidShootIntake() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(
                        AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
                                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                                .alongWith(
                                        intakePath("Mid-Driveback")
                                                .andThen(intakePath("MidDriveback-Depot"))
                                )
                );
        return auto;
    }

    public Command MidStationDepotShootWhileMoving() {
        Command auto = AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
                .alongWith(AutoAimCommands.hoodAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose()))
                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                .alongWith(
                        intakePathResetOdometry("Mid-Station")
                                .andThen(intakePath("Station-Depot"))
                                .andThen(intakePath("Depot-Climb"))
                                .andThen(finalAutoAim())
                );
        return auto;
    }

    /* High Autos */

    public Command HighShootPreload() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(
                        AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
                                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                                .alongWith(
                                        intakePath("High-Driveback")
                                )
                );
        return auto;
    }

    public Command HighShootClimb() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(
                        AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose())
                                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                                .alongWith(
                                        intakePath("HighDriveback-Climb")
                                )
                );
        return auto;

    }
    public Command HighShootIntakeClimb() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose()))
                .alongWith(AutoAimCommands.autoIndex(robotContainer))
                .alongWith(
                        intakePath("High-Station")
                                .andThen(intakePath("Station-Climb"))
                );
        return auto;
    }


    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }
}