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
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class Autos {
    private DriveSubsystem drive;

    private RobotContainer robotContainer;
    private String firstPath;
    private static boolean choreo;
    public static final String LOW_X2_NEUTRAL_STATION = "Low x2 Neutral Station";
    public static final String LOW_X2_NEUTRAL = "Low x2 Neutral";
    public static final String LOW_NEUTRAL_LOOP = "Low Neutral Loop";
    public static final String MID_DEPOT_STATION = "Mid Depot Station";
    public static final String MID_BUMP_STATION = "Mid Bump Station";
    public static final String HIGH_X2_NEUTRAL = "High x2 Neutral";
    public static final String HIGH_NEUTRAL_LOOP = "High Neutral Loop";
    public static final String LOW_NEUTRAL_STATION = "Low Neutral Station";
    public static final String MID_BUMP_NOSTATION = "Mid Bump No Station";
    public static final String ONE_METER = "One Meter";

    public static Command lowNeutralLoop;
    public static Command highNeutralLoop;
    public static Command midDepotStation;
    public static Command midBumpStation;
    public static Command midBumpNoStation;
    public static Command oneMeter;

    private final String WAIT_SHOOT = "wait_shoot";
    private final String WAIT_SHOOT_EXTENDED = "wait_shoot_extended";
    private final String WAIT_NO_SHOOT = "wait_no_shoot";
    private final String RETURN_UNDER_TRENCH_HIGH = "return_under_trench_high";
    private final String RETURN_UNDER_TRENCH_LOW = "return_under_trench_low";
    private final String RETURN_UNDER_TRENCH_STATION = "return_under_trench_station";
    private final String BUMP_WAIT = "bump wait";

    /**
     * Returns the alliance-corrected starting pose of the first path in an auto
     * sequence. Used to pre-seed odometry while disabled.
     *
     * @param firstPathName the PathPlanner path file name (no extension)
     * @param useChoreo
     * @return the starting Pose2d, or empty if the path cannot be loaded
     */
    public static Optional<Pose2d> getStartPose(String firstPathName, boolean useChoreo) {
        PathPlannerPath path;
        try {
            if(useChoreo) path = PathPlannerPath.fromChoreoTrajectory(firstPathName);
            else path = PathPlannerPath.fromPathFile(firstPathName);
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
        midBumpStation = MidBumpStation();
        midBumpNoStation = MidBump();
        oneMeter = OneMeterAuto();
    }

    /* Path Commands */

    private Command intakeDrive(Command driveCommand) {
        return robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
                .alongWith(robotContainer.getHood().setPosition(() -> 0.0))
                .until(() -> robotContainer.getHood().getPosition() < 0.3)
//                .until(() -> robotContainer.getIntake().getDeployReady()
//                        && robotContainer.getHood().getPosition() < 0.3)
//                .withTimeout(0.1)
                .andThen(
                        driveCommand
                                .deadlineFor(
                                        robotContainer.getIntake().setIntakingAuto()
                                                .alongWith(robotContainer.getIndexer().setIndexerVoltageWithTowerSpeed(
                                                        () -> 0.0, () -> 0.0)
                                                // .alongWith(robotContainer.getTurret().setPositionSafe(() ->
                                                // SubsystemConstants.turretDefaultRadians,
                                                // () -> 0.0, () -> true))))
                                                 .alongWith(robotContainer.getTurret().aimAtHub())
                                                .alongWith(robotContainer.getHood().setPosition(() -> 0.0))))
                                                );
    }

    private Command intakePath(String pathName, boolean useChoreo) {
        PathPlannerPath path;
        try {
            if(useChoreo) path = PathPlannerPath.fromChoreoTrajectory(pathName);
            else path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;

        //var followPath = drive.followPathCommand(finalPath);
        var followPath = new FollowPathCommand(drive, ()-> finalPath, ()-> Alliance.get(),true);

        return intakeDrive(followPath);
    }

    private Command intakePathResetOdometry(String pathName, boolean useChoreo) {
        PathPlannerPath path;
        try {
            if(useChoreo) path = PathPlannerPath.fromChoreoTrajectory(pathName);
            else path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;

        Pose2d startPose = new Pose2d(
                finalPath.getStartingDifferentialPose().getTranslation(),
                finalPath.getIdealStartingState().rotation());

        return SuperstructureCommands.resetOdometry(robotContainer, startPoseSupplier(startPose))
                .andThen(intakePath(pathName, useChoreo));
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

    private Command bumpWaitCommand() {
        Supplier<Double> waitTime = () -> robotContainer.getBumpWaitTime();

        Supplier<Command> scoreCommandSupplier = () -> {
            if (SubsystemConstants.hasTurret) {
                return (AutoAimCommands.autoIndex(robotContainer)
                        .alongWith(robotContainer.getShooter().autoAim())
                        .alongWith(robotContainer.getTurret().snapThenTrack())
                        .alongWith(robotContainer.getHood().autoAimUnsafe())
                        .alongWith(robotContainer.getIntake().oscillateIntake()));
            } else {
                return (AutoAimCommands.driveAutoAim(
                                robotContainer,
                                () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                () -> new ChassisSpeeds(0, 0, 0))
                        .alongWith(AutoAimCommands.autoIndex(robotContainer)
                                .alongWith(robotContainer.getShooter().autoAim())
                                .alongWith(robotContainer.getHood().autoAimUnsafe())
                                .alongWith(robotContainer.getIntake().oscillateIntake())));
            }
        };

        return Commands.defer(() -> {
            Logger.recordOutput("Drive/waitTime", waitTime.get());
            return scoreCommandSupplier.get().withTimeout(waitTime.get());
        }, scoreCommandSupplier.get().getRequirements());
    }


    public Command buildAuto(List<String> paths, boolean climb, boolean useChoreo) {
        Command pathCommands;

        if (paths.get(0).equals(BUMP_WAIT)) {
            pathCommands = SuperstructureCommands.resetOdometry(robotContainer, () -> getStartPose("MidBump", useChoreo).get())
                    .andThen(
                            bumpWaitCommand()
                    );
        }
        else pathCommands = intakePathResetOdometry(paths.get(0), useChoreo);

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
                pathCommands = pathCommands.andThen(Commands.waitSeconds(1.0));
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
            else if (paths.get(i).equals(BUMP_WAIT)) {
                bumpWaitCommand();
            }
            else {
                pathCommands = pathCommands.andThen(intakePath(paths.get(i), useChoreo));
            }
        }

        if (SubsystemConstants.hasTurret) {
            return pathCommands.andThen(
                                AutoAimCommands.autoIndex(robotContainer)
                                        .alongWith(robotContainer.getTurret().autoAim())
                                        .alongWith(robotContainer.getHood().autoAimUnsafe())
                                        .alongWith(robotContainer.getIntake().oscillateIntake())
                                        .alongWith(robotContainer.getShooter().autoAim()));
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


    public Command LowNeutralLoop() {
        choreo = true;
        String firstPath = "LowNeutral"; //"HighNeutralUnderTrench" for intake to start other way
        if (!SubsystemConstants.hasTurret) {
            return buildAuto(
                    List.of(firstPath, RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT, "LowScoreLoop",
                            RETURN_UNDER_TRENCH_LOW),
                    false, true);
        }
        return buildAuto(List.of(firstPath, RETURN_UNDER_TRENCH_LOW, WAIT_SHOOT_EXTENDED, "LowLoop",
                RETURN_UNDER_TRENCH_LOW), false, true);
    }

    public Command HighNeutralLoop() {
        choreo = true;
        String firstPath = "HighNeutral"; //"HighNeutralUnderTrench" for intake to start other way
        if (!SubsystemConstants.hasTurret) {
            return buildAuto(
                    List.of(firstPath, RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT, "HighScoreLoop",
                            RETURN_UNDER_TRENCH_HIGH),
                    false, true);
        }
        return buildAuto(List.of(firstPath, RETURN_UNDER_TRENCH_HIGH, WAIT_SHOOT_EXTENDED, "HighLoop",
                RETURN_UNDER_TRENCH_HIGH), false, true);
    }

    public Command MidBumpStation() {
        return buildAuto(List.of(BUMP_WAIT, "MidBump", "BumpLoop", "BumpMid", WAIT_SHOOT, "BumpStation", WAIT_NO_SHOOT, "StationScore"), false, true);
    }

    public Command MidBump() {
        return buildAuto(List.of(BUMP_WAIT, "MidBump", "BumpLoop", "BumpMid"), false, true);
    }

    public Command OneMeterAuto() {
        return (new WaitCommand(0.1)).andThen(intakePathResetOdometry("three meter", false));
    }

    public Command getOneMeter() {
        return oneMeter;
    }


    public Command getCommand(String autoName) {
        return switch (autoName) {
//            case LOW_NEUTRAL_LOOP_ANGED -> LowNeutralLoopAngled();
//            case LOW_X2_NEUTRAL_STATION -> LowDoubleNeutralStation();
//            case LOW_X2_NEUTRAL -> LowDoubleNeutral();
            case LOW_NEUTRAL_LOOP -> lowNeutralLoop;
            case MID_DEPOT_STATION -> midDepotStation;
//            case HIGH_X2_NEUTRAL -> HighDoubleNeutral();
            case HIGH_NEUTRAL_LOOP -> highNeutralLoop;
            case ONE_METER -> oneMeter;
            case MID_BUMP_STATION -> midBumpStation;
            case MID_BUMP_NOSTATION -> midBumpNoStation;
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
            case MID_BUMP_STATION -> "Mid-Bump";
            case MID_BUMP_NOSTATION -> "Mid-Bump";
            default -> "";
        };
    }
}