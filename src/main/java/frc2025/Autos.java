package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        RobotState.getInstance().setManualShuttleOverride(false);
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
        if (Robot.isSimulation()) {
            return new FollowPathCommand(drive, () -> finalPath, () -> true, true);
        }

        return new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true);
//        return deploy.andThen(new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true));
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
                .andThen(new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true));
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

        Command aim = robotContainer.getShooter().autoAim()
                .alongWith(robotContainer.getHood().autoAim());

        if (SubsystemConstants.hasTurret) {
            aim = aim.alongWith(robotContainer.getTurret().autoAim());
        }

        Command pathCommands = intakePathResetOdometry(paths.get(0))
                        .deadlineWith(robotContainer.getIntake().setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage));

        for(int i = 1; i < paths.size(); i++) {
            if(paths.get(i).equals("wait 2s")) {
                if(SubsystemConstants.hasTurret) {
                    pathCommands = pathCommands.andThen((AutoAimCommands.autoIndex(robotContainer)
                            .alongWith(robotContainer.getIntake().oscillateIntake())).withTimeout(2.5));
                }

                else {
                    pathCommands = pathCommands.andThen(
                            AutoAimCommands.driveAutoAim(
                                    robotContainer,
                                    () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                    () -> new ChassisSpeeds(0, 0, 0)
                            )
                            .alongWith(AutoAimCommands.autoIndex(robotContainer)
                            .alongWith(robotContainer.getIntake().oscillateIntake()))
                            .withTimeout(2.5)
                    );
                }
            }

            else {
                pathCommands = pathCommands.andThen(
                        robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                                .until(() -> robotContainer.getIntake().getDeployReady())
                                .andThen(
                                        intakePath(paths.get(i))
                                        .deadlineWith(
                                                robotContainer.intakeSubsystem.setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage)
                                                .alongWith(
                                                        robotContainer.indexerSubsystem.setVoltage(() -> 0.0)
                                                )
                                        )
                                )
                );
            }
        }

        if(SubsystemConstants.hasTurret) {
            return deploy.andThen(
                    aim.alongWith(pathCommands
                            .andThen(
                                    AutoAimCommands.autoIndex(robotContainer)
                                            .alongWith(robotContainer.getIntake().oscillateIntake())
                            )
                    )
            );
        } else {
            return deploy.andThen(
                    aim.alongWith(pathCommands.andThen(
                            AutoAimCommands.driveAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(), () -> new ChassisSpeeds(0, 0, 0))
                                    .alongWith(AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getIntake().oscillateIntake()))
                    ))
            );
        }
    }


    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }

    public Command LowStealLowNeutralLow() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Steal", "Steal-Low", "wait 2s", "Low-Neutral", "Neutral-Low"), false);
        }
        return buildAuto(List.of("Low-Steal", "Steal-LowScore", "wait 2s", "LowScore-Neutral", "Neutral-LowScore"), false);
    }
    public Command LowNeutralLowNeutralLow() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Neutral", "Neutral-Low", "wait 2s", "Low-NeutralMid", "NeutralMid-Low"),false);
        }
        return buildAuto(List.of("Low-Neutral", "Neutral-LowScore", "wait 2s", "LowScore-NeutralMid", "NeutralMid-LowScore"),false);
    }
    public Command LowStealLowNeutralStation() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Steal", "Steal-Low", "wait 2s", "Low-Neutral", "Neutral-Station"), false);
        }
        return buildAuto(List.of("Low-Steal", "Steal-LowScore", "wait 2s", "LowScore-Neutral", "Neutral-Station"), false);
    }
    public Command LowNeutralDeepStation() {
        return buildAuto(List.of("Low-NeutralMid", "NeutralMid-Low", "wait 2s", "Low-Station", "wait 2s", "Station-Score"), false);
//        return buildAuto(List.of("Low-NeutralMid", "NeutralMid-Low", "wait 2s", "Low-Station", "Station-Score"), false);
    }
    public Command LowStealStation() {
        return buildAuto(List.of("Low-Steal", "Steal-Station", "Station-Score"), false);
    }

    public Command MidStationDepotClimb() {
        return buildAuto(List.of("Mid-Station", "wait 2s", "Station-Depot", "Depot-Climb"), true);
    }

    public Command HighNeutralHighNeutralHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Neutral", "Neutral-High", "wait 2s", "High-NeutralMid", "NeutralMid-High"), false);
        }
        return buildAuto(List.of("High-Neutral", "Neutral-HighScore", "wait 2s", "HighScore-NeutralMid", "NeutralMid-HighScore"), false);
    }
    public Command HighStealHighNeutralHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Steal", "Steal-High", "wait 2s", "High-Neutral", "Neutral-High"), false);
        }
        return buildAuto(List.of("High-Steal", "Steal-HighScore", "wait 2s", "HighScore-Neutral", "Neutral-HighScore"), false);
    }

    public Command HighNeutralDeepDepot() {
        return buildAuto(List.of("High-NeutralMid", "NeutralMid-Depot", "Depot-Score"), false);
    }


    // High Cortisol Unused Autos
    public Command LowNeutralLowNeutralStation() {return buildAuto(List.of("Low-Neutral", "Neutral-Low", "wait 2s", "Low-NeutralMid", "NeutralMid-Station"), false); }
    public Command LowNeutralStation() { return buildAuto(List.of("Low-Neutral", "Neutral-Station", "Station-Score"), false); }
    public Command LowStationDepot() {return buildAuto(List.of("Low-Station", "Station-Depot", "Depot-Dump"), false);}
}