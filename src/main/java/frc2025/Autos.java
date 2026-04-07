package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import java.util.Optional;
import java.util.function.Supplier;

public class Autos {
    private DriveSubsystem drive;

    private RobotContainer robotContainer;
    private String firstPath;
    public static final String LOW_X2_NEUTRAL_STATION = "Low x2 Neutral Station";
    public static final String LOW_X2_NEUTRAL = "Low x2 Neutral";
    public static final String LOW_NEUTRAL_LOOP = "Low Neutral Loop";
    public static final String MID_DEPOT_STATION = "Mid Depot Station";
    public static final String HIGH_X2_NEUTRAL = "High x2 Neutral";
    public static final String HIGH_NEUTRAL_LOOP = "High Neutral Loop";
    public static final String LOW_NEUTRAL_STATION = "Low Neutral Station";
    public static final String ONE_METER = "One Meter";

    public static Command lowNeutralLoop;
    public static Command highNeutralLoop;
    public static Command midDepotStation;
    public static Command oneMeter;

    private final String WAIT_SHOOT = "wait_shoot";
    private final String WAIT_SHOOT_EXTENDED = "wait_shoot_extended";
    private final String WAIT_NO_SHOOT = "wait_no_shoot";
    private final String RETURN_UNDER_TRENCH_HIGH = "return_under_trench_high";
    private final String RETURN_UNDER_TRENCH_LOW = "return_under_trench_low";
    private final String RETURN_UNDER_TRENCH_STATION = "return_under_trench_station";
    private final String INTAKE_RETRACT_PATH = "intake retract path";

    /**
     * Returns the alliance-corrected starting pose of the first path in an auto
     * sequence. Used to pre-seed odometry while disabled.
     *
     * @param firstPathName the PathPlanner path file name (no extension)
     * @return the starting Pose2d, or empty if the path cannot be loaded
     */
    public static Optional<Pose2d> getStartPose(String firstPathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(firstPathName);
            Pose2d pose = new Pose2d(
                    path.getStartingDifferentialPose().getTranslation(),
                    path.getIdealStartingState().rotation());
            if (Alliance.get()) {
                pose = FlippingUtil.flipFieldPose(pose);
            }
            return Optional.of(pose);
        } catch (IOException | ParseException e) {
            return Optional.empty();
        }
    }

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrive();
        System.out.println(this.robotContainer);

        lowNeutralLoop = LowNeutralLoop();
        highNeutralLoop = HighNeutralLoop();
        midDepotStation = MidStationDepot();
        oneMeter = OneMeterAuto();
    }

    /* Path Commands */

    private Command intakeDrive(Command driveCommand) {
        return robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                .alongWith(robotContainer.getHood().setPosition(() -> 0.0))
                .until(() -> robotContainer.getIntake().getDeployReady()
                        && robotContainer.getHood().getPosition() < 0.3)
                .withTimeout(1.0)
                .andThen(
                        driveCommand
                                .deadlineFor(
                                        robotContainer.getIntake().setIntakingAuto()
                                                .alongWith(robotContainer.getIndexer().setIndexerVoltageWithTowerSpeed(
                                                        () -> 0.0, () -> 0.0)
                                                // .alongWith(robotContainer.getTurret().setPositionSafe(() ->
                                                // SubsystemConstants.turretDefaultRadians,
                                                // () -> 0.0, () -> true))))
                                                 .alongWith(robotContainer.getTurret().aimAtHub())))
                                                );
    }

    private Command intakePathBuiltin(String pathName) {
        PathPlannerPath path;

        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        var followPath = robotContainer.getDrive().followPathBuiltinCommand(pathName);

        return intakeDrive(followPath);

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
                finalPath.getIdealStartingState().rotation());

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
        // .alongWith(robotContainer.getTurret().aimAtHub());

        Command pathCommands = intakePathResetOdometry(paths.get(0));

        for (int i = 1; i < paths.size(); i++) {
            if (paths.get(i).equals(WAIT_SHOOT) || paths.get(i).equals(WAIT_SHOOT_EXTENDED)) {
                double waitTime = (paths.get(i).equals(WAIT_SHOOT)) ? 2.5 : 4.0;
                if (SubsystemConstants.hasTurret) {
                    pathCommands = pathCommands.andThen(
                            (AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getShooter().autoAim())
                                    .alongWith(robotContainer.getTurret().snapThenTrack())
                                    .alongWith(robotContainer.getHood().autoAimUnsafe())
                                    .alongWith(robotContainer.getIntake().oscillateIntake()))
                                    .withTimeout(waitTime));
                } else {
                    pathCommands = pathCommands.andThen(
                            AutoAimCommands.driveAutoAim(
                                    robotContainer,
                                    () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                    () -> new ChassisSpeeds(0, 0, 0))
                                    .alongWith(AutoAimCommands.autoIndex(robotContainer)
                                            .alongWith(robotContainer.getShooter().autoAim())
                                            .alongWith(robotContainer.getHood().autoAimUnsafe())
                                            .alongWith(robotContainer.getIntake().oscillateIntake()))
                                    .withTimeout(waitTime));
                }
            } else if (paths.get(i).equals(WAIT_NO_SHOOT)) {
                pathCommands = pathCommands.andThen(
                        Commands.waitSeconds(1.0));
            } else if (paths.get(i).equals(RETURN_UNDER_TRENCH_HIGH)) {
                Supplier<Pose2d> preTrench = () -> {
                    Pose2d pose = FieldConstants.redTrenchHighMid;
                    if (!Alliance.get()) {
                        pose = mirrorPose(pose);
                    }
                    return pose;
                };

                Supplier<Pose2d> endPose = () -> {
                    Pose2d pose = FieldConstants.blueTrenchHighZone;
                    if (Alliance.get()) pose = mirrorPose(pose);
                    return pose;
                };

                Supplier<String> returnPathSupplier = () -> {
                    String path = "Trench-High";
                    if (!SubsystemConstants.hasTurret)
                        path = "Trench-HighScore";
                    return path;
                };

                pathCommands = pathCommands.andThen(intakeDrive(
                        PathPlanCommands.returnUnderTrenchAuto(robotContainer, preTrench, endPose, () -> 0.0)));
//                        .andThen(intakePath(returnPathSupplier.get()));
            } else if (paths.get(i).equals(RETURN_UNDER_TRENCH_LOW)
                    || paths.get(i).equals(RETURN_UNDER_TRENCH_STATION)) {
                Supplier<Pose2d> preTrench = () -> {
                    Pose2d pose = FieldConstants.redTrenchLowMid;
                    if (!Alliance.get()) {
                        pose = mirrorPose(pose);
                    }
                    return pose;
                };

                boolean station = paths.get(i).equals(RETURN_UNDER_TRENCH_STATION);
                Supplier<String> returnPathSupplier = () -> {
                    String path = "Trench-Low";
                    if (!SubsystemConstants.hasTurret)
                        path = "Trench-LowScore";
                    if (station)
                        path = "Trench-Station";
                    return path;
                };

                Supplier<Pose2d> endPose = () -> {
                    Pose2d pose = FieldConstants.blueTrenchLowZone;
                    if (Alliance.get()) pose = mirrorPose(pose);
                    return pose;
                };

                Supplier<Double> endVelocity = () -> {
                    if (station)
                        return 2.5;
                    return 0.0;
                };
                pathCommands = pathCommands.andThen(intakeDrive(PathPlanCommands.returnUnderTrenchAuto(robotContainer,
                        preTrench, endPose, endVelocity)));
//                        .andThen(intakePath(returnPathSupplier.get()));
            }
//            else if (paths.get(i).equals(INTAKE_RETRACT_PATH)) {
//                pathCommands = pathCommands.andThen(
//                        robotContainer.getTurret().setPositionSafe(() -> SubsystemConstants.turretDefaultRadians, () -> 0.0, () -> true).alongWith(
//                                robotContainer.getIntake().setDeployPosition(() -> 2.5)
//                        ).until(() -> (Math.abs(robotContainer.getTurret().getPositionRadians()-SubsystemConstants.turretDefaultRadians) < 0.1
//                                && (robotContainer.getIntake().getDeployPosition() < 3.1)))
//                        .andThen(
//                                intakePath(paths.get(i+1))
//                                        .alongWith(
//                                                robotContainer.getIntake().setIntakingAuto()
//                                        )
//                        )
//                );
//                i++;
//            }
            else {
                pathCommands = pathCommands.andThen(intakePath(paths.get(i)));
            }
        }

        if (SubsystemConstants.hasTurret) {
            return (new WaitCommand(0.1)).andThen(
                        pathCommands
                        .andThen(
                                AutoAimCommands.autoIndex(robotContainer)
                                        .alongWith(robotContainer.getTurret().autoAim())
                                        .alongWith(robotContainer.getHood().autoAimUnsafe())
                                        .alongWith(robotContainer.getIntake().oscillateIntake())
                                        .alongWith(robotContainer.getShooter().autoAim()))
            );
        } else {
            return (new WaitCommand(0.1)).andThen(
                    pathCommands
                    .andThen(
                            AutoAimCommands
                                    .driveAutoAim(robotContainer,
                                            () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                            () -> new ChassisSpeeds(0, 0, 0))
                                    .alongWith(robotContainer.getHood().autoAimUnsafe())
                                    .alongWith(AutoAimCommands.autoIndex(robotContainer)
                                    .alongWith(robotContainer.getIntake().oscillateIntake()))
                                    .alongWith(robotContainer.getShooter().autoAim())));
        }
    }

//    public Command LowNeutralStation() {
//        return buildAuto(List.of("Low-NeutralMid (under trench)", RETURN_UNDER_TRENCH_STATION),
//                false);
//    }
//
//    public Command LowNeutralLoopAngled() {
//        return buildAuto(List.of("Low-Neutral-Angled (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-Loop",
//                RETURN_UNDER_TRENCH_LOW), false);
//    }
//
//    public Command LowDoubleNeutralStation() {
//        if (!SubsystemConstants.hasTurret) {
//            return buildAuto(
//                    List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "LowScore-NeutralMid",
//                            RETURN_UNDER_TRENCH_STATION),
//                    false);
//        }
//        return buildAuto(List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-NeutralMid",
//                RETURN_UNDER_TRENCH_STATION), false);
//    }
//
//    public Command LowDoubleNeutral() {
//        if (!SubsystemConstants.hasTurret) {
//            return buildAuto(
//                    List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "LowScore-NeutralMid",
//                            RETURN_UNDER_TRENCH_LOW),
//                    false);
//        }
//        return buildAuto(List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "Low-NeutralMid",
//                RETURN_UNDER_TRENCH_LOW), false);
//    }

    public Command LowNeutralLoop() {
        if (!SubsystemConstants.hasTurret) {
            return buildAuto(
                    List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "LowScore-Loop",
                            RETURN_UNDER_TRENCH_LOW),
                    false);
        }
        return buildAuto(List.of("Low-Neutral (under trench)", RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT_EXTENDED, "Low-Loop",
                RETURN_UNDER_TRENCH_LOW), false);
    }

    public Command MidStationDepot() {
        return buildAuto(List.of("Mid-Station", WAIT_SHOOT, "Station-Depot", "Depot-Sweep"),
                false);
    }

    public Command HighNeutralLoop() {
        if (!SubsystemConstants.hasTurret) {
            return buildAuto(
                    List.of("High-Neutral (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "HighScore-Loop",
                            RETURN_UNDER_TRENCH_HIGH),
                    false);
        }
        return buildAuto(List.of("High-Neutral (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT_EXTENDED, "High-Loop",
                RETURN_UNDER_TRENCH_HIGH), false);
    }

    public Command OneMeterAuto() {
        return (new WaitCommand(0.1)).andThen(intakePathResetOdometry("three meter"));
    }

    public Command getOneMeter() {
        return oneMeter;
    }

//    public Command HighDoubleNeutral() {
//        if (!SubsystemConstants.hasTurret) {
//            return buildAuto(
//                    List.of("High-Neutral (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "HighScore-NeutralMid",
//                            RETURN_UNDER_TRENCH_HIGH),
//                    false);
//        }
//        return buildAuto(List.of("High-Neutral (under trench)", RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "High-NeutralMid",
//                RETURN_UNDER_TRENCH_HIGH), false);
//    }
//
//    public Command LowPath() {
//        return Commands.waitSeconds(0.01).andThen(intakePathResetOdometry("Low-Neutral (under trench)"));
//    }

    public Command getCommand(String autoName) {
        return switch (autoName) {
//            case LOW_NEUTRAL_LOOP_ANGLED -> LowNeutralLoopAngled();
//            case LOW_X2_NEUTRAL_STATION -> LowDoubleNeutralStation();
//            case LOW_X2_NEUTRAL -> LowDoubleNeutral();
            case LOW_NEUTRAL_LOOP -> lowNeutralLoop;
            case MID_DEPOT_STATION -> midDepotStation;
//            case HIGH_X2_NEUTRAL -> HighDoubleNeutral();
            case HIGH_NEUTRAL_LOOP -> highNeutralLoop;
            case ONE_METER -> oneMeter;
//            case LOW_NEUTRAL_STATION -> LowNeutralStation();
            default -> Commands.none();
        };
    }

    public String getFirstPath(String autoName) {
        return switch (autoName) {
            case LOW_X2_NEUTRAL_STATION, LOW_X2_NEUTRAL, LOW_NEUTRAL_LOOP -> "Low-Neutral (under trench)";
            case LOW_NEUTRAL_STATION, "Low-NeutralMid (under trench)" -> "Low-NeutralMid (under trench)";
            case MID_DEPOT_STATION -> "Mid-Depot";
            case HIGH_X2_NEUTRAL, HIGH_NEUTRAL_LOOP -> "High-Neutral (under trench)";
            default -> "";
        };
    }
}