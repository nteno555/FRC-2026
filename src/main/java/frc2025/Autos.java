package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.FollowPathCommand;
import frc2025.commands.PathPlanCommands;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.FieldConstants;
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
    private final String WAIT_SHOOT = "wait_shoot";
    private final String WAIT_SHOOT_EXTENDED = "wait_shoot_extended";
    private final String WAIT_NO_SHOOT = "wait_no_shoot";
    private final String RETURN_UNDER_TRENCH_HIGH = "return_under_trench_high";
    private final String RETURN_UNDER_TRENCH_LOW = "return_under_trench_low";
    private final String RETURN_UNDER_TRENCH_STATION = "return_under_trench_station";

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrive();
        System.out.println(this.robotContainer);
    }

    /* Path Commands */

    private Command intakeDrive(Command driveCommand) {
        return robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                                .alongWith(robotContainer.getHood().setPosition(() -> 0.0))
                                                .until(() -> robotContainer.getIntake().getDeployReady() && robotContainer.getHood().getPosition() < 0.3)
                                .withTimeout(1.0)
                                .andThen(
                                        driveCommand
                                        .deadlineFor(
                                                robotContainer.getIntake().setIntakingAuto()
                                                .alongWith(robotContainer.getIndexer().setVoltage(() -> SubsystemConstants.indexerReverseVoltage)
                                                .alongWith(robotContainer.getTurret().aimAtHub()))
                                        )
                                );
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

        var followPath = new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true);

        return intakeDrive(followPath);
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
        return FlippingUtil.flipFieldPose(pose);
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
//                .alongWith(robotContainer.getTurret().aimAtHub());

        Command pathCommands = intakePathResetOdometry(paths.get(0));

        for(int i = 1; i < paths.size(); i++) {
            if(paths.get(i).equals(WAIT_SHOOT) || paths.get(i).equals(WAIT_SHOOT_EXTENDED)) {
                double waitTime = (paths.get(i).equals(WAIT_SHOOT)) ? 2.0 : 3.0;
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
            } else if(paths.get(i).equals(RETURN_UNDER_TRENCH_HIGH)) {
                Supplier<Pose2d> endPoseSupplier = () -> {
                    Pose2d pose = FieldConstants.blueTrenchHighReturn;
                    var alliance = DriverStation.Alliance.Red;
                    if(DriverStation.getAlliance().isPresent()) alliance = DriverStation.getAlliance().get();
                    if (alliance == DriverStation.Alliance.Red) {
                        pose = mirrorPose(pose);
                    }
                    return pose;
                };
                pathCommands = pathCommands.andThen(intakeDrive(PathPlanCommands.returnUnderTrenchAuto(robotContainer, endPoseSupplier, () -> 0.0)));
            } else if(paths.get(i).equals(RETURN_UNDER_TRENCH_LOW) || paths.get(i).equals(RETURN_UNDER_TRENCH_STATION)) {
                Supplier<Pose2d> endPoseSupplier = () -> {
                    Pose2d pose = FieldConstants.blueTrenchLowReturn;
                    var alliance = DriverStation.Alliance.Red;
                    if (DriverStation.getAlliance().isPresent()) alliance = DriverStation.getAlliance().get();
                    if (alliance == DriverStation.Alliance.Red) {
                        pose = mirrorPose(pose);
                    }
                    return pose;
                };

                boolean station = paths.get(i).equals(RETURN_UNDER_TRENCH_STATION);
                Supplier<Double> endVelocity = () -> {
                    if (station) return 2.5;
                    return 0.0;
                };
                pathCommands = pathCommands.andThen(intakeDrive(PathPlanCommands.returnUnderTrenchAuto(robotContainer, endPoseSupplier, endVelocity)));
            } else {
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



    public Command LowNeutralStation() {
        if (SubsystemConstants.hasTurret) {
            return buildAuto(List.of("Low-NeutralMid (under trench)", RETURN_UNDER_TRENCH_STATION, "Low-Station"), false);
        }
        return buildAuto(List.of("Low-NeutralMid (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "LowScore-Station", WAIT_NO_SHOOT, "Station-Score"), false);
    }

    public Command LowDoubleNeutralStation() {
        return buildAuto(List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-NeutralMid", RETURN_UNDER_TRENCH_STATION, "Low-Station"), false);
    }

    public Command LowDoubleNeutral() {
        return buildAuto(List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-NeutralMid", RETURN_UNDER_TRENCH_LOW), false);
    }

    public Command LowNeutralClose() {
        return buildAuto(List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-Close", RETURN_UNDER_TRENCH_LOW), false);
    }

    public Command LowNeutralLoop() {
        return buildAuto(List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-Loop", RETURN_UNDER_TRENCH_LOW), false);
    }

    public Command LowDoubleLoop() {
        return buildAuto(List.of("Low-Loop (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-Loop", RETURN_UNDER_TRENCH_LOW), false);
    }

    public Command MidStationDepotClimb() {
        return buildAuto(List.of("Mid-Station", WAIT_SHOOT, "Station-Depot", "Depot-Climb"), true);
    }

    public Command HighNeutralLoop() {
        return buildAuto(List.of("High-Neutral (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "High-Loop", RETURN_UNDER_TRENCH_HIGH), false);
    }

    public Command HighDoubleNeutral() {
        return buildAuto(List.of("High-Neutral (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "High-NeutralMid", RETURN_UNDER_TRENCH_HIGH), false);
    }

    public Command HighNeutralClose() {
        return buildAuto(List.of("High-Neutral (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "High-Close", RETURN_UNDER_TRENCH_HIGH), false);
    }

    public Command HighDoubleLoop() {
        return buildAuto(List.of("High-Loop (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "High-Loop", RETURN_UNDER_TRENCH_HIGH), false);
    }
}