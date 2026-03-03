package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.FollowPathCommand;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.util.Alliance;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

public class Autos {
    private DriveSubsystem drive;

    private RobotContainer robotContainer;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrive();
        System.out.println(this.robotContainer);
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

        var followPath = new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true);

        return robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                .alongWith(robotContainer.getHood().setPosition(() -> 0.0))
                                .until(() -> robotContainer.getIntake().getDeployReady() && robotContainer.getHood().getPosition() < 0.3)
                                .andThen(
                                        followPath
                                        .deadlineFor(
                                                robotContainer.intakeSubsystem.setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage)
                                                .alongWith(robotContainer.indexerSubsystem.setVoltage(() -> 0.0))
                                        )
                                );
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

        return SuperstructureCommands.resetOdometry(robotContainer, startPoseSupplier(startPose))
                .andThen(intakePath(pathName));
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
        Command deploy = robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                .until(() -> robotContainer.getIntake().getDeployReady());

        Command aim = robotContainer.getShooter().autoAim();

        if (SubsystemConstants.hasTurret) {
            aim = aim.alongWith(robotContainer.getTurret().autoAim());
        }

        Command pathCommands = intakePathResetOdometry(paths.get(0));

        for(int i = 1; i < paths.size(); i++) {
            if(paths.get(i).equals("wait shoot")) {
                if(SubsystemConstants.hasTurret) {
                    pathCommands = pathCommands.andThen(
                            (AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getHood().autoAimUnsafe())
                                    .alongWith(robotContainer.getIntake().oscillateIntake()))
                                    .withTimeout(2.5)
                    );
                } else {
                    pathCommands = pathCommands.andThen(
                            AutoAimCommands.driveAutoAim(
                                            robotContainer,
                                            () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                            () -> new ChassisSpeeds(0, 0, 0)
                                    )
                                    .alongWith(AutoAimCommands.autoIndex(robotContainer)
                                            .alongWith(robotContainer.getHood().autoAimUnsafe())
                                            .alongWith(robotContainer.getIntake().oscillateIntake()))
                                    .withTimeout(2.5)
                    );
                }
            } else if (paths.get(i).equals("wait no shoot")) {
                pathCommands = pathCommands.andThen(
                        Commands.waitSeconds(2.5)
                );
            } else {
                pathCommands = pathCommands.andThen(intakePath(paths.get(i)));
            }
        }

        if(SubsystemConstants.hasTurret) {
            return aim.alongWith(pathCommands
                    .andThen(
                            AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getHood().autoAimUnsafe())
                                    .alongWith(robotContainer.getIntake().oscillateIntake())
                    ));
        } else {
            return aim.alongWith(pathCommands
                    .andThen(
                            AutoAimCommands.driveAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(), () -> new ChassisSpeeds(0, 0, 0))
                                    .alongWith(robotContainer.getHood().autoAimUnsafe())
                                    .alongWith(AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getIntake().oscillateIntake()))
                    ));
        }
    }



    public Command LowStealLowNeutralStation() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Steal", "Steal-Low", "wait shoot", "Low-Neutral", "Neutral-Station"), false);
        }
        return buildAuto(List.of("Low-Steal", "Steal-LowScore", "wait shoot", "LowScore-Neutral", "Neutral-Station"), false);
    }
    public Command LowNeutralDeepStation() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-NeutralMid", "NeutralMid-Low", "wait shoot", "Low-Station", "wait no shoot", "Station-Score"), false);
        }
        return buildAuto(List.of("Low-NeutralMid", "NeutralMid-Low", "wait shoot", "Low-Station", "wait no shoot", "Station-Score"), false);
    }

    public Command LowStealLowLoopLow() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Steal", "Steal-Low", "wait shoot", "Low-NeutralMidLoop-Low"), false);
        }
        return buildAuto(List.of("Low-Steal", "Steal-Low", "wait shoot", "Low-NeutralMidLoop-LowScore"), false);
    }

    public Command MidStationDepotClimb() {
        return buildAuto(List.of("Mid-Station", "wait shoot", "Station-Depot", "Depot-Climb"), true);
    }

    public Command HighNeutralHighNeutralHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Neutral", "Neutral-High", "wait shoot", "High-NeutralMid", "NeutralMid-High"), false);
        }
        return buildAuto(List.of("High-Neutral", "Neutral-HighScore", "wait shoot", "HighScore-NeutralMid", "NeutralMid-HighScore"), false);
    }
    public Command HighStealHighNeutralHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Steal", "Steal-High", "wait shoot", "High-Neutral", "Neutral-High"), false);
        }
        return buildAuto(List.of("High-Steal", "Steal-HighScore", "wait shoot", "HighScore-Neutral", "Neutral-HighScore"), false);
    }

    public Command HighStealHighLoopHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Steal", "Steal-High", "wait shoot", "High-NeutralMidLoop-High"), false);
        }
        return buildAuto(List.of("High-Steal", "Steal-High", "wait shoot", "High-NeutralMidLoop-HighScore"), false);
    }

    public Command HighStealHighLoopHigh_StartUnderTrench() {
         if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Steal (under trench)", "Steal-High", "wait shoot", "High-NeutralMidLoop-High"), false);
        }
        return buildAuto(List.of("High-Steal (under trench)", "Steal-High", "wait shoot", "High-NeutralMidLoop-HighScore"), false);
    }

    public Command LowStealLowNeutralStation_StartUnderTrench() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Steal (under trench)", "Steal-Low", "wait shoot", "Low-Neutral", "Neutral-Station"), false);
        }
        return buildAuto(List.of("Low-Steal (under trench)", "Steal-LowScore", "wait shoot", "LowScore-Neutral", "Neutral-Station"), false);
    }
}