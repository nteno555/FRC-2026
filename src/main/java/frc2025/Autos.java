package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.FollowPathCommand;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
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

    public static Pose2d startPose = HIGH_START;
    private RobotContainer robotContainer;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrive();
    }

    public Command autoScore(Pose2d pose) {
        Command score = //AutoAimCommands.shooterAutoSpinUp(robotContainer, () -> pose)
            AutoAimCommands.autoAimCommand(robotContainer, () -> pose, () -> new ChassisSpeeds(0,0,0))
//                .andThen(
//                        new WaitUntilCommand(() -> {
//                            double vx = drive.getFieldRelativeSpeeds().vxMetersPerSecond;
//                            double vy = drive.getFieldRelativeSpeeds().vyMetersPerSecond;
//                            double velocity = Math.hypot(vx, vy);
//                            return robotContainer.shooterSubsystem.getShotReady() && velocity < SubsystemConstants.kMaxShootVelocity;
//                        })
                    .withTimeout(2.0)
                                .andThen(
                                        SuperstructureCommands.shootCommand(robotContainer)
                                                .alongWith(new WaitCommand(2.0))
                                );
        return score;
    }

    private Command intakeSWM(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;

        return
                new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true, true, robotContainer)
                        .andThen(new WaitCommand(0.2));
    }

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
                .alongWith(robotContainer.getIntake().setVoltage(() -> 5.0))
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

    private void setShuttling(boolean shuttle) {
        if (RobotState.getInstance().getShuttling() != shuttle) {
            RobotState.getInstance().setShuttling(shuttle);
        }
    }

    public Command LowShootPreload() {
        setShuttling(false);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(LOW_START))
                .andThen(intakePath("Low-Driveback"))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command MidShootPreload() {
        setShuttling(false);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(intakePath("Mid-Driveback"))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command MidShootWhileMoving() {
        setShuttling(false);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(intakeSWM("Mid-Driveback"));
        return auto;
    }

    public Command HighShootPreload() {
        setShuttling(false);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(intakePath("High-Driveback"))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command MidShootIntake() {
        setShuttling(false);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(intakePath("Mid-Driveback"))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()))
                .andThen(intakePath("MidDriveback-Depot"))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command HighShootClimb() {
        setShuttling(false);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()))
                .andThen(intakePath("HighDriveback-Climb"));
        return auto;

    }
    public Command HighShootIntakeClimb() {
        setShuttling(false);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command LowShuttleAuto() {
        setShuttling(true);
        Command auto = drive.resetOdometryCommand(startPoseSupplier(LOW_START))
                    .andThen(intakePath("Low-Neutral"))
                    .andThen(intakeSWM("Neutral-Driveback").alongWith(robotContainer.getIntake().setVoltage(() -> -5.0)));
        return auto;
    }

    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }
}