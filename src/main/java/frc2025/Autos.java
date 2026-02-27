package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
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
        RobotState.getInstance().setShuttling(false);

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

    private Command autoShoot() {
        return AutoAimCommands.autoShoot(
                robotContainer,
                () -> RobotState.getInstance().getTargetPose(),
                () -> new ChassisSpeeds(0, 0, 0)
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
        Command auto =
                robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                        .until(() -> robotContainer.getIntake().getDeployReady())
                        .andThen(
                                robotContainer.getIntake().setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage)
                                        .alongWith(robotContainer.getShooter().autoAim())
                                        .alongWith(robotContainer.getHood().autoAim())
                        );

        if (SubsystemConstants.hasTurret) {
                auto = auto.alongWith(robotContainer.getTurret().autoAim());
        }

        Command pathCommands = intakePathResetOdometry(paths.get(0));

        for(int i = 1; i < paths.size(); i++) {
            if(paths.get(i).equals("wait 2s")) {
                if(SubsystemConstants.hasTurret) {
                    pathCommands = pathCommands.andThen(
                            AutoAimCommands.autoIndex(robotContainer)
                                            .withTimeout(2.5)
                    );
                } else {
                    pathCommands = pathCommands.andThen(
                            AutoAimCommands.driveAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(), () -> new ChassisSpeeds(0, 0, 0))
                                    .alongWith(AutoAimCommands.autoIndex(robotContainer))
                                    .withTimeout(2.5)
                    );
                }
            }
            else if (paths.get(i).contains("Dump") && SubsystemConstants.hasTurret) {
                pathCommands = pathCommands.andThen(intakePath(paths.get(i)).alongWith(AutoAimCommands.autoIndex(robotContainer)));
            }

            else {
                pathCommands = pathCommands.andThen(intakePath(paths.get(i)));
            }
        }

        if(SubsystemConstants.hasTurret) {
            return auto.alongWith(pathCommands.andThen(AutoAimCommands.autoIndex(robotContainer)));
        } else {
            return auto.alongWith(pathCommands.andThen(
            AutoAimCommands.driveAutoAim(robotContainer, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(), () -> new ChassisSpeeds(0, 0, 0))
                                    .alongWith(AutoAimCommands.autoIndex(robotContainer))));
        }
    }


    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }

    public Command LowStealLowNeutralLow() {return buildAuto(List.of("Low-Steal (no turret)", "Steal-Low (no turret)", "wait 2s", "Low-Neutral (no turret)", "Neutral-Low (no turret)"), false);}
    public Command LowNeutralLowNeutralLow() {return buildAuto(List.of("Low-Neutral", "Neutral-Low", "wait 2s", "Low-NeutralMid", "NeutralMid-Low"),false);}
    public Command LowStealLowNeutralStation() { return buildAuto(List.of("Low-Steal", "Steal-Low", "wait 2s", "Low-Neutral", "Neutral-Station"), false); }
    public Command LowNeutralStation() { return buildAuto(List.of("Low-Neutral", "Neutral-Station"), false); }
    public Command LowStationDepot() {return buildAuto(List.of("Low-Station", "Station-Depot", "Depot-Dump"), false);}

    public Command MidStationDepotClimb() { return buildAuto(List.of("Mid-Station", "wait 2s", "Station-Depot", "Depot-Climb"), true); }

    public Command HighNeutralHighNeutralHigh() {return buildAuto(List.of("High-Neutral", "Neutral-High", "wait 2s", "High-NeutralMid", "NeutralMid-High"), false);}
    public Command HighStealHighNeutralHigh() { return buildAuto(List.of("High-Steal", "Steal-High", "wait 2s", "High-Neutral", "Neutral-High"), false); }
    public Command HighStealHighNeutralClimb() { return buildAuto(List.of("High-Steal", "Steal-HighBack", "wait 2s", "HighBack-Neutral", "Neutral-Climb (Trench High)"), true); }
    public Command HighNeutralDepot() {return buildAuto(List.of("High-Neutral", "Neutral-Depot", "Depot-Dump"), false);}
}