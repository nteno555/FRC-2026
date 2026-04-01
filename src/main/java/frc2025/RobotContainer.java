package frc2025;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc2025.subsystems.drive.DriveConstants.BackLeft;
import static frc2025.subsystems.drive.DriveConstants.BackRight;
import static frc2025.subsystems.drive.DriveConstants.DrivetrainConstants;
import static frc2025.subsystems.drive.DriveConstants.FrontLeft;
import static frc2025.subsystems.drive.DriveConstants.FrontRight;
import static frc2025.subsystems.drive.DriveConstants.kSpeedAt12Volts;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveIOReal;
import frc2025.subsystems.drive.DriveIOSim;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.hood.HoodIOSim;
import frc2025.subsystems.indexer.IndexerIOReal;
import frc2025.subsystems.indexer.IndexerIOSim;
import frc2025.subsystems.indexer.IndexerSubsystem;
import frc2025.subsystems.intake.IntakeIOReal;
import frc2025.subsystems.intake.IntakeIOSim;
import frc2025.subsystems.intake.IntakeSubsystem;
import frc2025.subsystems.hood.HoodIOReal;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.subsystems.shooter.ShooterIOReal;
import frc2025.subsystems.shooter.ShooterIOSim;
import frc2025.subsystems.shooter.ShooterSubsystem;
import frc2025.subsystems.turret.TurretIOReal;
import frc2025.subsystems.turret.TurretIOSim;
import frc2025.subsystems.turret.TurretSubsystem;
import frc2025.subsystems.vision.VisionIOReal;
import frc2025.subsystems.vision.VisionIOSim;
import frc2025.subsystems.vision.VisionSubsystem;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
        public static final double DEAD_BAND_DRIVER = 0.1;
        public static final double DEAD_BAND_OPERATOR = 0.1;
        private final CommandXboxController driverXboxController = new CommandXboxController(0);
        private final CommandXboxController operatorXboxController = new CommandXboxController(1);
        private final CommandXboxController testSystemsController = new CommandXboxController(2);
        private final CommandXboxController tuningController = new CommandXboxController(3);
        private final CommandXboxController driveTuningController = new CommandXboxController(4);
        private final SendableChooser<String> autoChooser = new SendableChooser<>();
        private String lastAutoName = "";
        private final Timer matchTime = new Timer();
        private boolean autoWarmupDone = false;

        DriveSubsystem driveSubsystem;

        Autos autos;
        Command autoCommand;

        private double driveSpeed = 0.8;
        private double slowmodeDriveSpeed = 0.3;
        private double rotationSpeed = 0.7;
        private double slowmodeRotationSpeed = 0.5;
        private boolean slowmode;
        private double manualTurretSpeed = 2.0;
        private boolean sysCheck;
        private double rpsOffset;

        private AimMode aimMode;
        private ManualAimState manualAimState;
        private boolean aiming;
        private boolean autoWarmupActive;

        private final Field2d field = new Field2d();

        private enum AimMode {
                TURRET,
                FIXED,
                MANUAL
        }

        private enum ManualAimState {
                FLUSH,
                SHUTTLE
        }

        public RobotContainer() {
                createSubsystems();
                configureBindings();
                configureAutoCommands();
                loggerInit();

                rpsOffset = 0.0;
        }

        public void periodic() {
//                updateMatchTime();
//                loggerPeriodic();
        }

        // shooterSubststem.setDefaultCommand( just shoot me);
        // - Yuna 2026
        private void createSubsystems() {
                if (RobotBase.isReal()) {
                        driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight,
                                        BackLeft, BackRight));
                } else {
                        driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight,
                                        BackLeft, BackRight));
                }
        }

        public void configureAutoCommands() {
                autoWarmupActive = false;

                autos = new Autos(this);
                autoChooser.setDefaultOption("No Auto", "No Auto");

//                autoChooser.addOption(Autos.LOW_NEUTRAL_LOOP_ANGLED, Autos.LOW_NEUTRAL_LOOP_ANGLED);
//                autoChooser.addOption(Autos.LOW_X2_NEUTRAL_STATION, Autos.LOW_X2_NEUTRAL_STATION);
//                autoChooser.addOption(Autos.LOW_X2_NEUTRAL, Autos.LOW_X2_NEUTRAL);
                autoChooser.addOption(Autos.LOW_NEUTRAL_LOOP, Autos.LOW_NEUTRAL_LOOP);
//                autoChooser.addOption(Autos.MID_STATION_DEPOT, Autos.MID_STATION_DEPOT);
//                autoChooser.addOption(Autos.HIGH_X2_NEUTRAL, Autos.HIGH_X2_NEUTRAL);
                autoChooser.addOption(Autos.HIGH_NEUTRAL_LOOP, Autos.HIGH_NEUTRAL_LOOP);
                autoChooser.addOption(Autos.ONE_METER, Autos.ONE_METER);

                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        public DriveSubsystem getDrive() {
                return driveSubsystem;
        }


        private void configureBindings() {
                slowmode = false;
                aiming = false;

                if (SubsystemConstants.hasTurret) {
                        aimMode = AimMode.TURRET;
                } else {
                        aimMode = AimMode.FIXED;
                }

                manualAimState = ManualAimState.FLUSH;

                driveSubsystem.setDefaultCommand(
                                // driveSubsystem.driveTeleop(
                                // () -> (driveSpeed) * handleDeadBand(getDriverLeftY(), DEAD_BAND_DRIVER, true)
                                // * kSpeedAt12Volts.in(MetersPerSecond),
                                // () -> (driveSpeed) * handleDeadBand(getDriverLeftX(), DEAD_BAND_DRIVER, true)
                                // * kSpeedAt12Volts.in(MetersPerSecond),
                                // () -> (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX(),
                                // DEAD_BAND_DRIVER, true) * Math.PI * 2
                                // )
                                driveSubsystem.driveTeleop(
                                                () -> (driveSpeed)
                                                                * handleDeadBand(driverXboxController.getLeftY()
                                                                                + tuningController.getLeftY()
                                                                                + testSystemsController.getLeftY(),
                                                                                DEAD_BAND_DRIVER, true)
                                                                * kSpeedAt12Volts.in(MetersPerSecond),
                                                () -> (driveSpeed)
                                                                * handleDeadBand(driverXboxController.getLeftX()
                                                                                + tuningController.getLeftX()
                                                                                + testSystemsController.getLeftX(),
                                                                                DEAD_BAND_DRIVER, true)
                                                                * kSpeedAt12Volts.in(MetersPerSecond),
                                                () -> (-rotationSpeed) * handleDeadBand(
                                                                driverXboxController.getRightX()
                                                                                + tuningController.getRightX()
                                                                                + testSystemsController.getRightX(),
                                                                DEAD_BAND_DRIVER, true) * Math.PI * 2));

                                                                                                                      // turret
                                                                                                                      // ll

                driverXboxController.a().onTrue(
                                SuperstructureCommands.resetOdometry(
                                                this,
                                                () -> !Alliance.get()
                                                                ? new Pose2d(new Translation2d(),
                                                                                new Rotation2d(Math.PI))
                                                                : new Pose2d()));

                driverXboxController
                                .b()
                                .and(driverXboxController.back().negate())
                                .onTrue(Commands.runOnce(() -> aiming = false));

                driverXboxController
                                .x()
                                .and(driverXboxController.back().negate())
                                .onTrue(Commands.runOnce(() -> aiming = true));

        }

        public static double handleDeadBand(double value, double deadBand, boolean scale) {
                if (scale) {
                        if (Math.abs(value) > Math.abs(deadBand)) {
                                double val = Math.abs(value) - Math.abs(deadBand);
                                val = val / (1.0 - Math.abs(deadBand));
                                value = val * Math.signum(value);
                                return value;
                        } else {
                                return 0.0;
                        }
                } else {
                        return (Math.abs(value) > Math.abs(deadBand)) ? value : 0.0;
                }
        }

        public void updateMatchTime() {
                /*
                 * 0:00 - 0:20 Auto
                 * 0:20 - 0:30 Transition
                 * 0:30 - 0:55 Shift 1
                 * 0:55 - 1:20 Shift 2
                 * 1:20 - 1:45 Shift 3
                 * 1:45 - 2:10 Shift 4
                 * 2:10 - 2:40 Endgame
                 */
                double now = matchTime.get();
                SmartDashboard.putNumber("MatchState/Elapsed", (int) now);
                if (DriverStation.isAutonomousEnabled()) {
                        SmartDashboard.putBoolean("MatchState/Countdown", (true));
                        SmartDashboard.putString("MatchState/State", "Auto");
                        SmartDashboard.putNumber("MatchState/Remaining Time", Math.round(20.0 - now));
                } else if (DriverStation.isTeleopEnabled()) {
                        if (now < 10.0) {
                                SmartDashboard.putNumber("MatchState/Remaining Time", Math.round(10 - now));
                                SmartDashboard.putBoolean("MatchState/Countdown", (now % 1.0 < 0.5));
                                SmartDashboard.putString("MatchState/State", "Transition");
                        } else if (now > 110.0) {
                                SmartDashboard.putString("MatchState/State", "Endgame");
                                SmartDashboard.putBoolean("MatchState/Countdown", (true));
                                SmartDashboard.putNumber("MatchState/Remaining Time", Math.round(140 - now));
                        } else {
                                SmartDashboard.putString("MatchState/State", "Shift");
                                now = (now - 10) % 25;
                                SmartDashboard.putNumber("MatchState/Remaining Time", Math.round(25 - now));
                                if (25 - now < 10) {
                                        SmartDashboard.putBoolean("MatchState/Countdown", (now % 1.0 < 0.5));
                                } else {
                                        SmartDashboard.putBoolean("MatchState/Countdown", (true));
                                }
                        }
                }
        }

        public void updateHubMessage() {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

                if (alliance.isEmpty())
                        return;

                double matchTime = DriverStation.getMatchTime();
                String gameData = DriverStation.getGameSpecificMessage();

                if (gameData.isEmpty()) {
                        SmartDashboard.putString("DriverStation/TimeToSwitch", "WAITING FOR FMS");
                        return;
                }

                boolean redInactiveFirst = false;
                switch (gameData.charAt(0)) {
                        case 'R' -> redInactiveFirst = true;
                        case 'B' -> redInactiveFirst = false;
                        default -> {
                                return;
                        }
                }

                boolean shift1Active = switch (alliance.get()) {
                        case Red -> !redInactiveFirst;
                        case Blue -> redInactiveFirst;
                };

                int matchTimeInt = (int) (Math.round(matchTime));

                if (matchTime > 130) {
                        if (shift1Active)
                                SmartDashboard.putString("DriverStation/TimeToSwitch", "ACTIVE");
                        else
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "OFF IN " + (matchTimeInt - 130) + "s");
                } else if (matchTime > 105) {
                        if (shift1Active)
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "OFF IN " + (matchTimeInt - 105) + "s");
                        else
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "ACTIVE IN " + (matchTimeInt - 105) + "s");
                } else if (matchTime > 80) {
                        if (!shift1Active)
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "OFF IN " + (matchTimeInt - 80) + "s");
                        else
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "ACTIVE IN " + (matchTimeInt - 80) + "s");
                } else if (matchTime > 55) {
                        if (shift1Active)
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "OFF IN " + (matchTimeInt - 55) + "s");
                        else
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "ACTIVE IN " + (matchTimeInt - 55) + "s");
                } else if (matchTime > 30) {
                        if (!shift1Active)
                                SmartDashboard.putString("DriverStation/TimeToSwitch", "HUB ACTIVE");
                        else
                                SmartDashboard.putString("DriverStation/TimeToSwitch",
                                                "ACTIVE IN " + (matchTimeInt - 30) + "s");
                } else
                        SmartDashboard.putString("DriverStation/TimeToSwitch", "HUB ACTIVE");
        }

        public void autoInit() {
                CommandScheduler.getInstance().cancelAll();
                autoWarmupActive = false;
                CommandScheduler.getInstance().schedule(autoCommand);
                matchTime.reset();
                matchTime.start();
        }

        public void loggerInit() {
                SmartDashboard.putData("Field", field);
        }

        public void loggerPeriodic() {
                field.setRobotPose(RobotState.getInstance().getVisionPose());

                updateHubMessage();

                boolean activeNow = RobotState.getInstance().isHubActive(false);
                boolean isNearActive = RobotState.getInstance().isHubActive(true);
                SmartDashboard.putBoolean("DriverStation/hubActive", activeNow || isNearActive);

                Logger.recordOutput("Superstructure/aimMode", aimMode);
                Logger.recordOutput("Superstructure/manualAimState", manualAimState);

        }

        public double getDriverLeftX() {
                if (driverXboxController.getLeftX() < 0) {
                        return -Math.pow(Math.abs(driverXboxController.getLeftX()), 1.5);
                }
                return Math.pow(driverXboxController.getLeftX(), 1.5);
        }

        public double getDriverLeftY() {
                if (driverXboxController.getLeftY() < 0) {
                        return -Math.pow(Math.abs(driverXboxController.getLeftY()), 1.5);
                }
                return Math.pow(driverXboxController.getLeftY(), 1.5);
        }

        public void teleopInit() {
                // autoCommand.cancel();

                CommandScheduler.getInstance().cancelAll();
                autoWarmupActive = false;
        }

        public void disabledPeriodic() {
//                Logger.recordOutput("Robot/disabledHeartbeat", Math.random());

                String selected = autoChooser.getSelected();
                if (selected == null)
                        selected = "No Auto";

//                if (!selected.equals(lastAutoName)) {
//                        String firstPath = autos.getFirstPath(selected);
//                        if (!firstPath.isEmpty()) {
//                                Autos.getStartPose(firstPath).ifPresent(pose -> {
//                                        driveSubsystem.resetOdometry(pose);
//                                        visionSubsystem.resetOdometry();
//                                });
//                        }
//                        lastAutoName = selected;
//                }

                autoCommand = autos.getCommand(selected);

//                if(!autoWarmupActive) {
//                        autoCommand.schedule();
//                        autoWarmupActive = true;
//                }
        }

        public void disabledInit() {
                if(!autoWarmupDone) {
                        Command autoWarmup = autos.LowNeutralLoop();
                        autoWarmup = autoWarmup.ignoringDisable(true);
                        autoWarmup.schedule();
                        autoWarmupDone = true;
                }
        }
}
