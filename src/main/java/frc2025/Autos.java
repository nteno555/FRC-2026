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
import frc2025.config.FieldConstants;
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

    public Command autoSpinUp(Supplier<Pose2d> robotPose) {
        double distance = robotPose.get().getTranslation().getDistance(RobotState.getInstance().getShootingWhileMovingVirtualPose().getTranslation());
        double wantedSpeed = ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(distance);
        return robotContainer.getShooter().setSpeed(() -> wantedSpeed);
    }


    private Command intakePath(String pathName, boolean shootWhileMove) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;

        return new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true, shootWhileMove, robotContainer)
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

    public Command LowShootPreload() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(LOW_START))
                .andThen(intakePath("Low-Driveback", false))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command MidShootPreload() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(intakePath("Mid-Driveback", false))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command MidShootWhileMoving() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(
                        autoSpinUp(() -> RobotState.getInstance().getVisionPose())
                                .alongWith(
                    intakePath("Mid-SWM1", false)
                    .andThen(intakePath("Mid-SWM2", true))
                    .andThen(() -> {
                        if (robotContainer.getIndexer().getHasPiece()) {autoScore(RobotState.getInstance().getVisionPose());}
                    })
                                )
                );
        return auto;
    }

    public Command HighShootPreload() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(intakePath("High-Driveback", false))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command MidShootIntake() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(MID_START))
                .andThen(intakePath("Mid-Driveback", false))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()))
                .andThen(intakePath("MidDriveback-Depot", false))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command HighShootClimb() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()))
                .andThen(intakePath("HighDriveback-Climb", false));
        return auto;

    }
    public Command HighShootIntakeClimb() {
        Command auto = drive.resetOdometryCommand(startPoseSupplier(HIGH_START))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()))
                .andThen(autoScore(RobotState.getInstance().getTargetPose()));
        return auto;
    }

    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }
}