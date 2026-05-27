package frc2025;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc2025.subsystems.drive.DriveConstants.BackLeft;
import static frc2025.subsystems.drive.DriveConstants.BackRight;
import static frc2025.subsystems.drive.DriveConstants.DrivetrainConstants;
import static frc2025.subsystems.drive.DriveConstants.FrontLeft;
import static frc2025.subsystems.drive.DriveConstants.FrontRight;
import static frc2025.subsystems.drive.DriveConstants.kSpeedAt12Volts;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc2025.commands.AutoAimCommands;
import frc2025.commands.SuperstructureCommands;
// import frc2025.config.ShooterTreeMap;
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
        // private final CommandXboxController operatorXboxController = new CommandXboxController(1);
        // private final CommandXboxController testSystemsController = new CommandXboxController(2);
        // private final CommandXboxController tuningController = new CommandXboxController(3);
        // private final CommandXboxController driveTuningController = new CommandXboxController(4);
        private final SendableChooser<String> autoChooser = new SendableChooser<>();
        private String lastAutoName = "";
        private final Timer matchTime = new Timer();
        private boolean autoWarmupDone = false;
        private boolean autoWon = false;

        DriveSubsystem driveSubsystem;
        VisionSubsystem visionSubsystem;
        IndexerSubsystem indexerSubsystem;
        IntakeSubsystem intakeSubsystem;
        HoodSubsystem hoodSubsystem;
        ShooterSubsystem shooterSubsystem;
        TurretSubsystem turretSubsystem;

        Autos autos;
        Command autoCommand;

//        private double flywheelRps = 42.0;
//        private double flyhoodAngle = 0.9;
        private final double[] listFlywheelRps = {20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 58.0};
        private final double[] listFlywheelAngle = {0.8, 1.2, 1.6, 2.0, 2.4};
        private int flywheelRpsIdx = 2;
        private int flywheelAngleIdx = 1;

        private final double triggerThreshold = 0.2;

        // private double driveSpeed = 1.3;
        private double slowmodeDriveSpeed = 0.3;
        // private double rotationSpeed = 1.2;
        private double slowmodeRotationSpeed = 0.5;
        private boolean slowmode;
        // private double manualTurretSpeed = 2.0;

        private double rpsOffset;
        private double devicesConnected;

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
                updateMatchTime();
                loggerPeriodic();
        }

        // shooterSubststem.setDefaultCommand( just shoot me);
        // - Yuna 2026
        private void createSubsystems() {
                if (RobotBase.isReal()) {
                        driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight,
                                        BackLeft, BackRight));
                        visionSubsystem = new VisionSubsystem(new VisionIOReal());
                        indexerSubsystem = new IndexerSubsystem(new IndexerIOReal());
                        intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());
                        hoodSubsystem = new HoodSubsystem(new HoodIOReal());
                        shooterSubsystem = new ShooterSubsystem(new ShooterIOReal());
                        turretSubsystem = new TurretSubsystem(new TurretIOReal());
                } else {
                        driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight,
                                        BackLeft, BackRight));
                        visionSubsystem = new VisionSubsystem(new VisionIOSim());
                        indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
                        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
                        hoodSubsystem = new HoodSubsystem(new HoodIOSim());
                        shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
                        turretSubsystem = new TurretSubsystem(new TurretIOSim());
                }
        }

        public void configureAutoCommands() {
                autoWarmupActive = false;

                autos = new Autos(this);
                autoChooser.setDefaultOption("No Auto", "No Auto");

                autoChooser.addOption(Autos.LOW_NEUTRAL_LOOP, Autos.LOW_NEUTRAL_LOOP);
                autoChooser.addOption(Autos.MID_BUMP_STATION, Autos.MID_BUMP_STATION);
                autoChooser.addOption(Autos.MID_BUMP_NOSTATION, Autos.MID_BUMP_NOSTATION);
                autoChooser.addOption(Autos.HIGH_NEUTRAL_LOOP, Autos.HIGH_NEUTRAL_LOOP);
                autoChooser.addOption(Autos.ONE_METER, Autos.ONE_METER);

                SmartDashboard.putData("Auto Chooser", autoChooser);
                SmartDashboard.putNumber("Bump Wait Time", 2.0);
        }

        public DriveSubsystem getDrive() {
                return driveSubsystem;
        }

        public IndexerSubsystem getIndexer() {
                return indexerSubsystem;
        }

        public IntakeSubsystem getIntake() {
                return intakeSubsystem;
        }

        public HoodSubsystem getHood() {
                return hoodSubsystem;
        }

        public ShooterSubsystem getShooter() {
                return shooterSubsystem;
        }

        public VisionSubsystem getVision() {
                return visionSubsystem;
        }

        public TurretSubsystem getTurret() {
                return turretSubsystem;
        }

        private void configureBindings() {

                // start: rezero drive
                // back: rezero intake
                // right stick: rezero turret
                // X: start shooting
                // B: stop shooting
                // Right Trigger: increase flywheel speed
                // Left Trigger: decrease flywheel speed
                // Right Bumper: decrease shooting angle
                // Left Bumper: increase shooting angle
                // D-pad up: intake on
                // D-pad down: intake off
                // D-pad left: rotate turret ccw
                // D-pad right: rotate turret cw

                slowmode = true;
                aiming = false;
                aimMode = AimMode.MANUAL;
                manualAimState = ManualAimState.FLUSH;

                intakeSubsystem.setDefaultCommand(intakeSubsystem.noMovement());
                hoodSubsystem.setDefaultCommand(hoodSubsystem.setPosition(() -> 0.1));
                shooterSubsystem.setDefaultCommand(shooterSubsystem.setVoltage(() -> 0.0));
                indexerSubsystem.setDefaultCommand(indexerSubsystem.setIndexerVoltageWithTowerSpeed(() -> 0.0, () -> 0.0));
                turretSubsystem.setDefaultCommand(turretSubsystem.setPositionSafe(() -> SubsystemConstants.turretDefaultRadians, () -> 0.0, () -> true));

                driveSubsystem.setDefaultCommand(
                        driveSubsystem.driveTeleop(
                                () -> (slowmodeDriveSpeed)
                                        * handleDeadBand(driverXboxController.getLeftY(),
                                        DEAD_BAND_DRIVER, true)
                                        * kSpeedAt12Volts.in(MetersPerSecond),
                                () -> (slowmodeDriveSpeed)
                                        * handleDeadBand(driverXboxController.getLeftX(),
                                        DEAD_BAND_DRIVER, true)
                                        * kSpeedAt12Volts.in(MetersPerSecond),
                                () -> (-slowmodeRotationSpeed) * handleDeadBand(
                                        driverXboxController.getRightX(),
                                        DEAD_BAND_DRIVER, true) * Math.PI * 2));

                driverXboxController.rightTrigger(triggerThreshold)
                        .onTrue(Commands.runOnce(() -> {
                            flywheelRpsIdx = Math.min(flywheelRpsIdx + 1, listFlywheelRps.length - 1);
                        }));

                driverXboxController.leftTrigger(triggerThreshold)
                        .onTrue(Commands.runOnce(() -> {
                            flywheelRpsIdx = Math.max(flywheelRpsIdx - 1, 0);
                        }));

                driverXboxController.rightBumper()
                        .onTrue(Commands.runOnce(() -> {
                            flywheelAngleIdx = Math.min(flywheelAngleIdx + 1, listFlywheelAngle.length - 1);
                        }));

                driverXboxController.leftBumper()
                        .onTrue(Commands.runOnce(() -> {
                            flywheelAngleIdx = Math.max(flywheelAngleIdx - 1, 0);
                        }));

                driverXboxController.button(8).onTrue(
                        SuperstructureCommands.resetOdometry(
                                this,
                                () -> !Alliance.get()
                                        ? new Pose2d(new Translation2d(),
                                        new Rotation2d(Math.PI))
                                        : new Pose2d()));

                driverXboxController.back().onTrue(
                        intakeSubsystem.homeThenRezero()
                );

                driverXboxController.rightStick()
                        .onTrue(
                                turretSubsystem.setPositionSafe(() -> 0.0, () -> 0.0, () -> true)
                        );

                driverXboxController
                        .x()
                        .and(driverXboxController.back().negate())
                        .onTrue(Commands.runOnce(() -> {
                            aiming = true;
                        }));

                driverXboxController
                        .b()
                        .and(driverXboxController.back().negate())
                        .onTrue(Commands.runOnce(() -> {
                            aiming = false;
                        }));

                driverXboxController.povUp() // left middle paddle
                        .onTrue(intakeSubsystem
                                .extendAndIntake());

                driverXboxController.povDown() // right bottom paddle
                        .onTrue(intakeSubsystem.
                                noMovement());

                driverXboxController.povLeft() // left bottom paddle
                        .whileTrue(
                                turretSubsystem.setVoltageWithClamp(() -> -2.0)
                        )
                        .whileFalse(
                                turretSubsystem.setVoltage(() -> 0.0)
                        );

                driverXboxController.povRight() // right middle paddle
                        .whileTrue(
                                turretSubsystem.setVoltageWithClamp(() -> 2.0)
                        )
                        .whileFalse(
                                turretSubsystem.setVoltage(() -> 0.0)
                        );

                new Trigger(() -> aiming)
                        .and(() -> aimMode == AimMode.MANUAL)
                        .and(() -> manualAimState == ManualAimState.FLUSH)
                        .and(() -> !(turretSubsystem.getSpinAround()))
                        .whileTrue(
                                shooterSubsystem.setSpeed(() -> listFlywheelRps[flywheelRpsIdx])
                                        .alongWith(hoodSubsystem.setPosition(
                                                () -> listFlywheelAngle[flywheelAngleIdx]))
                                        .alongWith(intakeSubsystem.oscillateIntake()));

                new Trigger(() -> aiming)
                        .and(() -> (aimMode == AimMode.MANUAL))
                        .whileTrue(indexerSubsystem.setIndexerVoltageWithTowerSpeed(
                                () -> SubsystemConstants.indexerShootVoltage, () -> 12.0));

                new Trigger(() -> turretSubsystem.getSpinAround()).whileTrue(shooterSubsystem.setVoltage(() -> 0.0));
//              driverXboxController
//                      .b()
//                      .and(driverXboxController.back())
//                      .whileTrue(hoodSubsystem.homeThenRezero());
//              driverXboxController
//                      .x()
//                      .and(driverXboxController.back())
//                      .whileTrue(intakeSubsystem.homeThenRezero());
//              driverXboxController
//                      .y()
//                      .and(driverXboxController.back())
//                      .whileTrue(turretSubsystem.rezeroCRT());

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

        private void updateMatchTime() {
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

        private void loggerPeriodic() {
                field.setRobotPose(RobotState.getInstance().getVisionPose());

                // updateHubMessage();

                boolean activeNow = RobotState.getInstance().isHubActive(false);
                boolean isNearActive = RobotState.getInstance().isHubActive(true);
                SmartDashboard.putBoolean("DriverStation/hubActive", activeNow || isNearActive);

                Logger.recordOutput("Superstructure/aimMode", aimMode);
                Logger.recordOutput("Superstructure/manualAimState", manualAimState);
                Logger.recordOutput("Superstructure/slowmode", slowmode);
                Logger.recordOutput("Superstructure/RPS Offset", rpsOffset);
                Logger.recordOutput("Superstructure/flywheelRps", flywheelRpsIdx);
                Logger.recordOutput("Superstructure/flywheelAngle", flywheelAngleIdx);

                Logger.recordOutput("Controller/LeftTrigger", driverXboxController.leftTrigger().getAsBoolean());
                Logger.recordOutput("Controller/RightTrigger", driverXboxController.rightTrigger().getAsBoolean());
                Logger.recordOutput("Controller/LeftBumper", driverXboxController.leftBumper().getAsBoolean());
                Logger.recordOutput("Controller/RightBumper", driverXboxController.rightBumper().getAsBoolean());
                Logger.recordOutput("Controller/A", driverXboxController.a().getAsBoolean());
                Logger.recordOutput("Controller/B", driverXboxController.b().getAsBoolean());
                Logger.recordOutput("Controller/X", driverXboxController.x().getAsBoolean());
                Logger.recordOutput("Controller/Y", driverXboxController.y().getAsBoolean());
                Logger.recordOutput("Controller/Start", driverXboxController.start().getAsBoolean());
                Logger.recordOutput("Controller/Back", driverXboxController.back().getAsBoolean());
                Logger.recordOutput("Controller/LeftStick", driverXboxController.leftStick().getAsBoolean());
                Logger.recordOutput("Controller/RightStick", driverXboxController.rightStick().getAsBoolean());
                Logger.recordOutput("Controller/PovUp", driverXboxController.povUp().getAsBoolean());
                Logger.recordOutput("Controller/PovDown", driverXboxController.povDown().getAsBoolean());
                Logger.recordOutput("Controller/PovLeft", driverXboxController.povLeft().getAsBoolean());
                Logger.recordOutput("Controller/PovRight", driverXboxController.povRight().getAsBoolean());
                Logger.recordOutput("Controller/RightY", driverXboxController.getRightY());
                Logger.recordOutput("Controller/RightX", driverXboxController.getRightX());
                Logger.recordOutput("Controller/RightTrigger", driverXboxController.getRightTriggerAxis());
                Logger.recordOutput("Controller/LeftTrigger", driverXboxController.getLeftTriggerAxis());

                for (int i = 1; i <= 10; i++) {
                    Logger.recordOutput("Controller/" + i, driverXboxController.button(i).getAsBoolean());
                }

                if (autoWon) {
                    Logger.recordOutput("Superstructure/Won Auto", "WON");
                }
                else {
                    Logger.recordOutput("Superstructure/Won Auto", "LOST");
                }

                double totalCurrent = driveSubsystem.getCurrent() + hoodSubsystem.getCurrent() + indexerSubsystem.getCurrent()
                        + indexerSubsystem.getCurrent() + intakeSubsystem.getCurrent() + shooterSubsystem.getCurrent() +
                        turretSubsystem.getCurrent();

                Logger.recordOutput("Current/Total Current", totalCurrent);
                Logger.recordOutput("Current/Battery Voltage", RobotController.getBatteryVoltage());
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
                shooterSubsystem.setSpeed(() -> 0.0);
                indexerSubsystem.setIndexerVoltageWithTowerSpeed(() -> 0.0, () -> 0.0);
                intakeSubsystem.setDeployVoltage(() -> 0.0);
                intakeSubsystem.setRollerVoltage(() -> 0.0);

//                AutoAimCommands.autoIndexTeleop(this, () -> true).schedule();

                matchTime.reset();
                matchTime.start();
        }

        public void disabledPeriodic() {
//                Logger.recordOutput("Robot/disabledHeartbeat", Math.random());

                String selected = autoChooser.getSelected();
                if (selected == null)
                        selected = "No Auto";

                // if (!selected.equals(lastAutoName)) {
                //         String firstPath = autos.getFirstPath(selected);
                //         if (!firstPath.isEmpty()) {
                //                 Autos.getStartPose(firstPath).ifPresent(pose -> {
                //                         SuperstructureCommands.resetOdometry(this, () -> pose);
                //                 });
                //         }
                //         lastAutoName = selected;
                // }

                autoCommand = Commands.none();

//                if(!autoWarmupActive) {
//                        autoCommand.schedule();
//                        autoWarmupActive = true;
//                }
        }

        public void disabledInit() {
                if(!autoWarmupDone) {
                        Command autoWarmup = autos.LowNeutralLoop();
                        autoWarmup = autoWarmup.ignoringDisable(true);
                        CommandScheduler.getInstance().schedule(autoWarmup);
                        // autoWarmup.schedule();
                        autoWarmupDone = true;
                }
        }

        public double getBumpWaitTime() {
            Logger.recordOutput("Drive/bumpWaitTimeSmartdashboard", SmartDashboard.getNumber("Bump Wait Time", 2.0));
            return SmartDashboard.getNumber("Bump Wait Time", 2.0);
        }
}
