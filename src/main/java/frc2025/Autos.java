package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
            return new FollowPathCommand(drive, () -> finalPath, () -> false, true);
            //.raceWith(robotContainer.getIntake().setIntaking(() -> true));
            //.andThen(new WaitCommand(0.2));
        }

        return new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true);
        //.raceWith(robotContainer.getIntake().setIntaking(() -> true));
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

        if (Robot.isSimulation()) {
            return  SuperstructureCommands.resetOdometry(robotContainer, () -> startPose)
                    .andThen(new FollowPathCommand(drive, () -> finalPath, () -> false, true));
        }

        return  SuperstructureCommands.resetOdometry(robotContainer, () -> startPose)
                .andThen(new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true));
                //.raceWith(robotContainer.getIntake().setIntaking(() -> true));
    }

    private Command finalAutoAim() {
                if (!SubsystemConstants.hasTurret) {
                        return AutoAimCommands.driveAutoAim(
                                robotContainer,
                                () -> RobotState.getInstance().getTargetPose(),
                                () -> new ChassisSpeeds(0,0,0)
                        ).alongWith(AutoAimCommands.autoIndex(robotContainer));
                } else {
                        //return AutoAimCommands.turretAutoAim(robotContainer, () -> RobotState.getInstance().getTargetPose(), null);
                    return Commands.none();
                }
    }

    private Command driveAutoAim() {
        return AutoAimCommands.driveAutoAim(
                robotContainer,
                () -> RobotState.getInstance().getTargetPose(),
                () -> new ChassisSpeeds(0,0,0)
        ).alongWith(AutoAimCommands.autoIndex(robotContainer));
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
        Command auto;
        if (SubsystemConstants.hasTurret) {
                auto = robotContainer.getTurret().autoAim()
                        .alongWith(robotContainer.getShooter().autoAim())
                        .alongWith(robotContainer.getHood().autoAim())
                        .alongWith(AutoAimCommands.autoIndex(robotContainer))
                        .alongWith(
                                robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                                        .until(() -> robotContainer.getIntake().getDeployReady())
                                                .andThen(robotContainer.getIntake().setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage)));
        } else {
                auto = robotContainer.getShooter().autoAim()
                        .alongWith(robotContainer.getHood().autoAim())
//                        .alongWith(AutoAimCommands.autoIndex(robotContainer))
                        .alongWith(
                                robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                                        .until(() -> robotContainer.getIntake().getDeployReady())
                                                .andThen(robotContainer.getIntake().setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage)));
        }
        

        Command pathCommands = intakePathResetOdometry(paths.get(0));

        for(int i = 1; i < paths.size(); i++) {
            if(paths.get(i).equals("wait 2s")) {
                if(SubsystemConstants.hasTurret) {
                    pathCommands = pathCommands.andThen(Commands.waitSeconds(2));
                } else {
                    pathCommands = pathCommands.andThen(
                            driveAutoAim()
                                    .withTimeout(2.5)
                    );
                }
            } else {
                pathCommands = pathCommands.andThen(intakePath(paths.get(i)));
            }
        }

        if(!SubsystemConstants.hasTurret) {
            return auto.alongWith(pathCommands.andThen(finalAutoAim()));
        } else {
            return auto.alongWith(pathCommands);
        }
    }


    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }

    public Command LowStationNeutralClimb() { return buildAuto(List.of("Low-Station", "Station-Neutral", "Neutral-Climb (Low)"), true); }
    public Command LowNeutralStationNeutralClimb() { return buildAuto(List.of("Low-Neutral", "Neutral-Station", "Station-Neutral", "Neutral-Climb (Low)"), true); }
    public Command LowNeutralStationClimb() {return buildAuto(List.of("Low-Neutral", "Neutral-Station", "Station-Climb"), true); }
    public Command LowStealLowNeutralClimb() {return buildAuto(List.of("Low-Steal", "Steal-Low", "wait 2s", "Low-Neutral", "Neutral-Climb (Low)"), true);}
    public Command LowStealLowNeutralStation() {return buildAuto(List.of("Low-Steal", "Steal-Low", "wait 2s", "Low-Neutral", "Neutral-Station"), false);}
    public Command LowStealLowNeutralLow() {return buildAuto(List.of("Low-Steal", "Steal-Low", "wait 2s", "Low-Neutral", "Neutral-Low"), false);}
    public Command LowNeutralLowNeutralMidLow() {return buildAuto(List.of("Low-Neutral", "Neutral-Low", "wait 2s", "Low-NeutralMid", "NeutralMid-Low"),false);}
    public Command LowStationDepotClimb() { return buildAuto(List.of("Low-Station", "Station-Depot", "Depot-Climb"), true); }
    public Command LowStationClimb() { return buildAuto(List.of("Low-Station", "Station-Climb"), true); }

    public Command MidClimb() { return buildAuto(List.of("Mid-Climb"), true); }
    public Command MidDepotNeutralClimb() {return buildAuto(List.of("Mid-Depot", "Depot-Neutral", "Neutral-Climb (Low)"), true);}
    public Command MidStationNeutralClimb() {return buildAuto(List.of("Mid-Station", "Station-Neutral", "Neutral-Climb (Low)"), true);}
    public Command MidStationDepotClimb() { return buildAuto(List.of("Mid-Station", "Station-Depot", "Depot-Climb"), true); }
    public Command MidStationDepot() {return buildAuto(List.of("Mid-Station", "Station-Depot"), false);}
    public Command MidStationDepotForward() {return buildAuto(List.of("Mid-Station", "Station-Depot", "Depot-Forward"), false);}

    public Command HighDepotStationClimb() { return buildAuto(List.of("High-Depot", "Depot-Station", "Station-Climb"), true); }
    public Command HighDepotNeutralClimb() { return buildAuto(List.of("High-Depot", "Depot-Neutral", "Neutral-Climb (High)"), true); }
    public Command HighDepotClimb() { return buildAuto(List.of("High-Depot", "Depot-Climb"), true); }
    public Command HighNeutralHighNeutralMid() {return buildAuto(List.of("High-Neutral", "Neutral-High", "wait 2s", "High-NeutralMid", "NeutralMid-High"), false);}
    public Command HighStealHighNeutralClimb() { return buildAuto(List.of("High-Steal", "Steal-HighBack", "wait 2s", "HighBack-Neutral", "Neutral-Climb (Trench High)"), true); }
}