package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.DriveCommands;
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
    private final String WAIT_SHOOT = "waitshoot";
    private final String WAIT_SHOOT_EXTENDED = "waitshootextended";
    private final String WAIT_NO_SHOOT = "waitnoshoot";
    private final String TRENCH_RETURN_HIGH = "trenchreturnhigh";
    private final String TRENCH_RETURN_LOW = "trenchreturnlow";

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
                                .withTimeout(1.0)
                                .andThen(
                                        followPath
                                        .deadlineFor(
                                                robotContainer.getIntake().setIntakingAuto()
                                                .alongWith(robotContainer.indexerSubsystem.setVoltage(() -> SubsystemConstants.indexerReverseVoltage))
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
        Command aim = robotContainer.getShooter().autoAim();

        Command pathCommands = intakePathResetOdometry(paths.get(0));

        for(int i = 1; i < paths.size(); i++) {
            if(paths.get(i).equals(WAIT_SHOOT) || paths.get(i).equals(WAIT_SHOOT_EXTENDED)) {
                double waitTime = (paths.get(i).equals(WAIT_SHOOT)) ? 3.0 : 4.0;
                if(SubsystemConstants.hasTurret) {
                    pathCommands = pathCommands.andThen(
                            (AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getTurret().autoAim())
                                    .alongWith(robotContainer.getHood().autoAimUnsafe())
                                    .alongWith(robotContainer.getIntake().oscillateIntake()))
                            .withTimeout(waitTime)
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
                                    .withTimeout(waitTime)
                    );
                }
            }
            else if (paths.get(i).equals(WAIT_NO_SHOOT)) {
                pathCommands = pathCommands.andThen(
                        Commands.waitSeconds(1.0)
                );
            }
            else if (paths.get(i).equals(TRENCH_RETURN_HIGH) || paths.get(i).equals(TRENCH_RETURN_LOW)) {
                pathCommands = pathCommands.andThen(
                        DriveCommands.autoTrenchReturn(robotContainer, paths.get(i).equals(TRENCH_RETURN_HIGH)));
//                        .alongWith(robotContainer.getIntake().setIntakingAuto())
//                        .alongWith(robotContainer.getIndexer().setVoltage(() -> SubsystemConstants.indexerReverseVoltage));
            }
            else {
                pathCommands = pathCommands.andThen(intakePath(paths.get(i)));
            }
        }

        if(SubsystemConstants.hasTurret) {
            return aim.alongWith(pathCommands
                    .andThen(
                            AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getTurret().autoAim())
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

    public Command LowNeutralDeepAutoTrenchStation() {
        return buildAuto(List.of("Low-NeutralMid (under trench)", TRENCH_RETURN_LOW, WAIT_SHOOT, "Trench-Station"), false);
    }

    public Command HighNeutralAutoTrench() {
        return buildAuto(List.of("High-Neutral (under trench)", TRENCH_RETURN_HIGH, WAIT_SHOOT), false);
    }

    public Command LowNeutralDeepStation() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-NeutralMid (under trench)", TRENCH_RETURN_LOW, WAIT_SHOOT, "Trench-Station", WAIT_NO_SHOOT, "Station-Score"), false);
//            return buildAuto(List.of("Low-NeutralMid (under trench)", "NeutralMid-Low", WAIT_SHOOT, "Low-Station", WAIT_NO_SHOOT, "Station-Score"), false);
        }
        return buildAuto(List.of("Low-NeutralMid (under trench)", TRENCH_RETURN_LOW, "Trench-LowScore", WAIT_SHOOT, "LowScore-Station", WAIT_NO_SHOOT, "Station-Score"), false);
//        return buildAuto(List.of("Low-NeutralMid (under trench)", "NeutralMid-LowScore", WAIT_SHOOT, "LowScore-Station", WAIT_NO_SHOOT, "Station-Score"), false);
    }

    public Command LowDoubleNeutral() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Neutral (under trench)", TRENCH_RETURN_LOW, WAIT_SHOOT, "LowTrench-NeutralMid", TRENCH_RETURN_LOW), false);
        }
        return buildAuto(List.of("Low-Neutral (under trench)", TRENCH_RETURN_LOW, "Trench-LowScore", WAIT_SHOOT, "LowScore-NeutralMid", TRENCH_RETURN_LOW, "Trench-LowScore"), false);
    }

    public Command LowNeutralClose() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Neutral (under trench)", TRENCH_RETURN_LOW, WAIT_SHOOT, "LowTrench-Close", TRENCH_RETURN_LOW), false);
        }
        return buildAuto(List.of("Low-NeutralMid (under trench", TRENCH_RETURN_LOW, "Trench-LowScore", WAIT_SHOOT, "LowScore-Close", TRENCH_RETURN_LOW, "Trench-LowScore"), false);
    }

    public Command LowStealLowNeutralDeep() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Steal (under trench)", "Steal-Low", WAIT_SHOOT, "Low-NeutralMid", "NeutralMid-Low"), false);
        }
        return buildAuto(List.of("Low-Steal (under trench)", "Steal-LowScore", WAIT_SHOOT, "LowScore-NeutralMid", "NeutralMid-LowScore"), false);
    }

    public Command LowStealLowLoopLow() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-Steal (under trench)", "Steal-Low", WAIT_SHOOT, "Low-NeutralMidLoop-Low"), false);
        }
        return buildAuto(List.of("Low-Steal (under trench)", "Steal-Low", WAIT_SHOOT, "LowScore-NeutralMidLoop-LowScore"), false);
    }

//    public Command LowNeutralLowNeutral() {
//        if (SubsystemConstants.hasTurret) {
//            return buildAuto(List.of("Low-Neutral (under trench)", "Neutral-Low", WAIT_SHOOT, "Low-NeutralMid", "NeutralMid-Low"), false);
//        }
//        return buildAuto(List.of("Low-Neutral (under trench)", "Neutral-LowScore", WAIT_SHOOT, "LowScore-NeutralMid", "NeutralMid-LowScore"), false);
//    }

    public Command MidStationDepotClimb() {
        return buildAuto(List.of("Mid-Station", WAIT_SHOOT, "Station-Depot", "Depot-Climb"), true);
    }

    public Command HighDoubleNeutral() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Neutral (under trench)", TRENCH_RETURN_HIGH, WAIT_SHOOT, "High-NeutralMid", TRENCH_RETURN_HIGH), false);
        }
        return buildAuto(List.of("High-Neutral (under trench)", TRENCH_RETURN_HIGH, "Trench-HighScore", WAIT_SHOOT, "HighScore-NeutralMid", TRENCH_RETURN_HIGH, "Trench-HighScore"), false);
    }

    public Command HighNeutralClose() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-NeutralMid (under trench)", TRENCH_RETURN_HIGH, WAIT_SHOOT, "High-Close", TRENCH_RETURN_HIGH), false);
        }
        return buildAuto(List.of("High-Neutral (under trench)", TRENCH_RETURN_HIGH, "Trench-HighScore", WAIT_SHOOT, "HighScore-Close", TRENCH_RETURN_HIGH, "Trench-HighScore"), false);
    }

    public Command HighNeutralHighNeutralHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Neutral (under trench)", "Neutral-High", WAIT_SHOOT, "High-NeutralMid", "NeutralMid-High"), false);
        }
        return buildAuto(List.of("High-Neutral (under trench)", "Neutral-HighScore", WAIT_SHOOT, "HighScore-NeutralMid", "NeutralMid-HighScore"), false);
    }
    public Command HighStealHighNeutralHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Steal (under trench)", "Steal-High", WAIT_SHOOT, "High-Neutral", "Neutral-High"), false);
        }
        return buildAuto(List.of("High-Steal (under trench)", "Steal-HighScore", WAIT_SHOOT, "HighScore-Neutral", "Neutral-HighScore"), false);
    }

    public Command HighStealHighNeutralDeep() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Steal (under trench)", "Steal-High", WAIT_SHOOT, "High-NeutralMid", "NeutralMid-High"), false);
        }
        return buildAuto(List.of("High-Steal (under trench)", "Steal-HighScore", WAIT_SHOOT, "HighScore-NeutralMid", "NeutralMid-HighScore"), false);
    }

    public Command HighStealHighLoopHigh() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("High-Steal (under trench)", "Steal-High", WAIT_SHOOT, "High-NeutralMidLoop-High"), false);
        }
        return buildAuto(List.of("High-Steal (under trench)", "Steal-High", WAIT_SHOOT, "HighScore-NeutralMidLoop-HighScore"), false);
    }


}