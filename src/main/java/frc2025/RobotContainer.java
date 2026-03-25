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
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final Timer matchTime = new Timer();

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    IndexerSubsystem indexerSubsystem;
    IntakeSubsystem intakeSubsystem;
    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

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
        Logger.recordOutput("Superstructure/aimMode", aimMode);
        Logger.recordOutput("Superstructure/manualAimState", manualAimState);
        Logger.recordOutput("Superstructure/RPS Offset", rpsOffset);
        updateMatchTime();
        loggerPeriodic();
    }
//shooterSubststem.setDefaultCommand( just shoot me);
    // - Yuna 2026
    private void createSubsystems() {
        if(RobotBase.isReal()) {
            driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOReal());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOReal());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());
            hoodSubsystem = new HoodSubsystem(new HoodIOReal());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOReal());
            turretSubsystem = new TurretSubsystem(new TurretIOReal());
        } else {
            driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOSim());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
            hoodSubsystem = new HoodSubsystem(new HoodIOSim());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
            turretSubsystem = new TurretSubsystem(new TurretIOSim());
        }
    }

    public void configureAutoCommands() {
        autos = new Autos(this);
        autoChooser.setDefaultOption("No Auto", new InstantCommand());

        if(SubsystemConstants.hasTurret) {
            autoChooser.addOption("Low x2 Neutral Station", autos.LowDoubleNeutralStation());
            autoChooser.addOption("Low x2 Neutral", autos.LowDoubleNeutral());
            autoChooser.addOption("Low Neutral Loop", autos.LowNeutralLoop());
            autoChooser.addOption("Low x2 Loop", autos.LowDoubleLoop());

            autoChooser.addOption("Mid Station Depot", autos.MidStationDepotClimb());

            autoChooser.addOption("High x2 Neutral", autos.HighDoubleNeutral());
            autoChooser.addOption("High x2 Loop", autos.HighDoubleLoop());
            autoChooser.addOption("High Neutral Loop", autos.HighNeutralLoop());

        } else {
            autoChooser.addOption("Low Neutral Station", autos.LowNeutralStation());

            autoChooser.addOption("Mid Station Depot", autos.MidStationDepotClimb());

        }

        SmartDashboard.putData("Auto Chooser", autoChooser);

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
        slowmode = false;
        aiming = false;

        if (SubsystemConstants.hasTurret) {
            aimMode = AimMode.TURRET;
        }
        else {
            aimMode = AimMode.FIXED;
        }

        manualAimState = ManualAimState.FLUSH;

        driveSubsystem.setDefaultCommand(
//                driveSubsystem.driveTeleop(
//                        () -> (driveSpeed) * handleDeadBand(getDriverLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                        () -> (driveSpeed) * handleDeadBand(getDriverLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                        () -> (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
//                )
                driveSubsystem.driveTeleop(
                        () -> (driveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY() + testSystemsController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (driveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX() + testSystemsController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX() + testSystemsController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                )
        );

        intakeSubsystem.setDefaultCommand(intakeSubsystem.setDeployVoltage(() -> 0.0));
        hoodSubsystem.setDefaultCommand(hoodSubsystem.setPosition(() -> 0.1));
        shooterSubsystem.setDefaultCommand(shooterSubsystem.setVoltage(() -> 0.0));
        indexerSubsystem.setDefaultCommand(indexerSubsystem.setVoltage(() -> 0.0));
//        turretSubsystem.setDefaultCommand(turretSubsystem.setVoltage(() -> 0.0));
        turretSubsystem.setDefaultCommand(turretSubsystem.setPositionSafe(() -> 0.0, () -> 0.0)); // for turret ll

        driverXboxController.a().onTrue(
                SuperstructureCommands.resetOdometry(
                        this,
                        () -> !Alliance.get()
                                ? new Pose2d(new Translation2d(), new Rotation2d(Math.PI))
                                : new Pose2d()
                )
        );

        driverXboxController
                .b()
                .and(driverXboxController.back().negate())
                .onTrue(Commands.runOnce(() -> aiming = false));

        driverXboxController
                .x()
                .and(driverXboxController.back().negate())
                .onTrue(Commands.runOnce(() -> aiming = true));


       new Trigger(() -> aiming)
               .and(() -> aimMode == AimMode.MANUAL)
               .and(() -> manualAimState == ManualAimState.FLUSH)
               .whileTrue(
                       shooterSubsystem.setSpeed(() -> SubsystemConstants.flushFlywheelRps)
                               .alongWith(hoodSubsystem.setPosition(() -> SubsystemConstants.flushHoodAngle))
                               .alongWith(intakeSubsystem.oscillateIntake())
               );
       new Trigger(() -> aiming)
               .and(() -> aimMode == AimMode.MANUAL)
               .and(() -> manualAimState == ManualAimState.SHUTTLE)
               .whileTrue(
                       shooterSubsystem.setSpeed(() -> ShooterTreeMap.getInstance().shuttlingRPSTreeMap.getInterpolated(3))
                               .alongWith(hoodSubsystem.setPosition(() -> SubsystemConstants.hoodMaxAngleRotations))
//                               .alongWith(intakeSubsystem.oscillateIntake())
               );

        new Trigger(() -> aiming)
                .and(() -> (aimMode == AimMode.FIXED))
                .whileTrue(
                        AutoAimCommands.driveAutoAim(
                                        this,
                                        () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                        () -> driveSubsystem.getTeleopSpeeds(
                                                (driveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                                (driveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                                (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                                        ))
                                .alongWith(shooterSubsystem.autoAim())
                                .alongWith(hoodSubsystem.autoAim())
//                                .alongWith(intakeSubsystem.oscillateIntake())
                );

        new Trigger(() -> aiming)
                .and(() -> (aimMode == AimMode.TURRET))
                .whileTrue(
                        shooterSubsystem.autoAim()
                                .alongWith(turretSubsystem.autoAim())
                                .alongWith(hoodSubsystem.autoAim())
                );

        new Trigger(() -> aiming)
                .and(() -> (aimMode == AimMode.TURRET || aimMode == AimMode.FIXED))
                .and(driverXboxController.y().negate())
                .whileTrue(AutoAimCommands.autoIndex(this, () -> aimMode == AimMode.TURRET));

        new Trigger(() -> aiming)
                .and(() -> (aimMode == AimMode.MANUAL))
                .and(driverXboxController.y().negate())
                .whileTrue(indexerSubsystem.setVoltage(() -> SubsystemConstants.indexerShootVoltage));

        driverXboxController
                .y()
                .and(driverXboxController.back().negate())
                .whileTrue(indexerSubsystem.setVoltage(() -> SubsystemConstants.indexerReverseVoltage));

        driverXboxController
                .b()
                .and(driverXboxController.back())
                .whileTrue(hoodSubsystem.homeThenRezero());
        driverXboxController
                .x()
                .and(driverXboxController.back())
                .whileTrue(intakeSubsystem.homeThenRezero());
         driverXboxController
                .y()
                .and(driverXboxController.back())
                .whileTrue(turretSubsystem.rezero());

        driverXboxController.povLeft() // left bottom paddle
                .onTrue(intakeSubsystem.setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage));
        driverXboxController.povDown() // right bottom paddle
                .onTrue(intakeSubsystem.setRollerVoltage(() -> 0.0));

        driverXboxController.rightBumper()
                .whileTrue(intakeSubsystem.oscillateIntake())
                .onFalse(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition));

        driverXboxController.rightTrigger()
                .onTrue(Commands.runOnce(() -> slowmode = !slowmode));

        driverXboxController.povUp() // left middle paddle
                .onTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition));
        driverXboxController.povRight() // right middle paddle
                .onTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeRetractPosition));


        operatorXboxController.leftBumper()
                .onTrue(Commands.runOnce(() -> aimMode = AimMode.TURRET));
        operatorXboxController.rightBumper()
                .onTrue(Commands.runOnce(() -> aimMode = AimMode.MANUAL));
        operatorXboxController.rightTrigger()
                .onTrue(Commands.runOnce(() -> aimMode = AimMode.FIXED));

        operatorXboxController.povUp()
                .onTrue(Commands.runOnce(() -> {
                    rpsOffset += 1.0;
                    ShooterTreeMap.getInstance().changeRPS(1.0);
                }));

        operatorXboxController.povDown()
                .onTrue(Commands.runOnce(() -> {
                    rpsOffset -= 1.0;
                    ShooterTreeMap.getInstance().changeRPS(-1.0);
                }));

        operatorXboxController.povLeft()
                .onTrue(Commands.runOnce(() -> {
                    rpsOffset = 0.0;
                    ShooterTreeMap.getInstance().resetRPS();
                }));

        operatorXboxController.y()
                .and(() -> (aimMode == AimMode.MANUAL))
                .onTrue(Commands.runOnce(() -> manualAimState = ManualAimState.FLUSH));
        operatorXboxController.x()
                .and(() -> (aimMode == AimMode.MANUAL))
                .onTrue(Commands.runOnce(() -> manualAimState = ManualAimState.SHUTTLE));

        new Trigger(() -> aimMode == AimMode.MANUAL || aimMode == AimMode.FIXED)
                .whileTrue(turretSubsystem.setVoltage(() -> manualTurretSpeed * operatorXboxController.getRightX()));

//        new Trigger(() -> slowmode).whileTrue(
//                driveSubsystem.driveTeleop(
//                        () -> (slowmodeDriveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                        () -> (slowmodeDriveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                        () -> (-slowmodeRotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
//                )
//                        .alongWith(intakeSubsystem.oscillateIntake())
//        );

        testSystemsController.x().onTrue(
                hoodSubsystem.setPosition(() -> 1.5)
        );
        testSystemsController.y().onTrue(
                hoodSubsystem.homeThenRezero()
        );
        testSystemsController.a().whileTrue(
                shooterSubsystem.setSpeed(() -> 15.0)
        );
        testSystemsController.b().whileTrue(
                indexerSubsystem.setVoltage(() -> -2.0)
        );
        testSystemsController.leftTrigger().whileTrue(
                intakeSubsystem.homeThenRezero()
        );
        testSystemsController.rightTrigger().onTrue(
                intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
        );
        testSystemsController.rightBumper().onTrue(
                intakeSubsystem.setRollerVoltage(() -> 4.0)
        );
        testSystemsController.leftBumper().onTrue(
                intakeSubsystem.setRollerVoltage(() -> 0.0)
        );
        testSystemsController.povLeft().whileTrue(
                turretSubsystem.setVoltage(() -> 1.5)
        );
        testSystemsController.povRight().whileTrue(
                turretSubsystem.setVoltage(() -> -1.5)
        );

        new Trigger(() -> (testSystemsController.isConnected())).whileTrue(
                turretSubsystem.setVoltage(() -> 0.0)
                        .alongWith(hoodSubsystem.setVoltage(() -> 0.0))
                        .alongWith(shooterSubsystem.setVoltage(() -> 0.0))
                        .alongWith(indexerSubsystem.setVoltage(() -> 0.0))
        );

//        driveTuningController.a()
//                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kForward));
//        driveTuningController.b()
//                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kReverse));
//        driveTuningController.x()
//                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
//        driveTuningController.y()
//                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));
//        driveTuningController.povLeft()
//                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kForward));
//        driveTuningController.povUp()
//                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kReverse));
//        driveTuningController.povDown()
//                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kForward));
//        driveTuningController.povRight()
//                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kReverse));


//    tuningController.leftBumper().whileTrue(
//            turretSubsystem.rezeroCRT()
//    );

    tuningController.povDown().whileTrue(
//            turretSubsystem.setVoltage(() -> -1.0);
            turretSubsystem.setPositionSafe(() -> 0.0, () -> 0.0)
    );

    tuningController.povRight().whileTrue(
            turretSubsystem.setVoltage(() -> 2.0)
//            turretSubsystem.setPositionSafe(() -> -Math.PI * 0.5, () -> 0.0)
    );

    tuningController.povUp().whileTrue(
            turretSubsystem.setPositionSafe(() -> Math.PI * 0.5, () -> 0.0)
    );

    tuningController.povLeft().whileTrue(
            turretSubsystem.setVoltage(() -> -2.0)
//            turretSubsystem.setPositionSafe(() -> Math.PI, () -> 0.0)
    );

//    tuningController.povLeft().whileTrue(
//            turretSubsystem.setVoltage(() -> 2.0)
//    ).whileFalse(turretSubsystem.setVoltage(() -> 0.0));
//
//    tuningController.povRight().whileTrue(
//            turretSubsystem.setVoltage(() -> -2.0)
//    ).whileFalse(turretSubsystem.setVoltage(() -> 0.0));
//
//    tuningController.povUp().whileTrue(
//            intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeRetractPosition)
//    )
//            .whileFalse(
//                    intakeSubsystem.setDeployVoltage(() -> 0.0)
//            );
//
//    tuningController.povDown().whileTrue(
//            intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition)
//    ).whileFalse(
//            intakeSubsystem.setDeployVoltage(() -> 0.0)
//    );
//
    tuningController.leftBumper().whileTrue(
//            turretSubsystem.autoAim()
            turretSubsystem.rezeroCRT()
    );


        tuningController.a().onTrue(
                SuperstructureCommands.resetOdometry(
                        this,
                        () -> !Alliance.get()
                                ? new Pose2d(new Translation2d(), new Rotation2d(Math.PI))
                                : new Pose2d()
                )
        );

        SmartDashboard.putNumber("Tuning/RPM", 0.0);
        SmartDashboard.putNumber("Tuning/hoodPosition", 0.0);
        tuningController.rightBumper()
                .whileTrue(
                        shooterSubsystem.setSpeed(() -> SmartDashboard.getNumber("Tuning/RPM", 0.0))
                                .alongWith(hoodSubsystem.setPosition(() -> SmartDashboard.getNumber("Tuning/hoodPosition", 0.0)))
//                                .alongWith(turretSubsystem.autoAim())
                )
                .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

//        tuningController.rightBumper()
//                .whileTrue(AutoAimCommands.autoAim(
//                        this,
//                        () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
//                        () -> driveSubsystem.getTeleopSpeeds(
//                                (-0.5) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                                (-0.5) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                                (0.7) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
//                        )
//                ))
//                .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

        tuningController.rightTrigger().whileTrue(
            indexerSubsystem.setVoltage(() -> SubsystemConstants.indexerShootVoltage))
            .whileFalse(indexerSubsystem.setVoltage(() -> 0.0));
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
        /* 0:00 - 0:20 Auto
        0:20 - 0:30 Transition
        0:30 - 0:55 Shift 1
        0:55 - 1:20 Shift 2
        1:20 - 1:45 Shift 3
        1:45 - 2:10 Shift 4
        2:10 - 2:40 Endgame */
        double now = matchTime.get();
        SmartDashboard.putNumber("MatchState/Elapsed", (int) now);
        if (DriverStation.isAutonomousEnabled()) {
            SmartDashboard.putBoolean("MatchState/Countdown", (true));
            SmartDashboard.putString("MatchState/State", "Auto");
            SmartDashboard.putNumber("MatchState/Remaining Time", Math.round(20.0 - now));
        }
        else if (DriverStation.isTeleopEnabled()) {
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

    public void updateHubMessage(){
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) return;
        
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
            default -> {return;}
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        int matchTimeInt = (int)(Math.round(matchTime));

            if (matchTime > 130) {
                if(shift1Active) SmartDashboard.putString("DriverStation/TimeToSwitch", "ACTIVE");
                else SmartDashboard.putString("DriverStation/TimeToSwitch", "OFF IN " + (matchTimeInt - 130) + "s");
            }
            else if (matchTime > 105) {
                if(shift1Active) SmartDashboard.putString("DriverStation/TimeToSwitch", "OFF IN " + (matchTimeInt - 105) + "s");
                else SmartDashboard.putString("DriverStation/TimeToSwitch", "ACTIVE IN " + (matchTimeInt - 105) + "s");
            }
            else if (matchTime > 80) {
                if(!shift1Active) SmartDashboard.putString("DriverStation/TimeToSwitch", "OFF IN " + (matchTimeInt - 80) + "s");
                else SmartDashboard.putString("DriverStation/TimeToSwitch", "ACTIVE IN " + (matchTimeInt - 80) + "s");
            }
            else if (matchTime > 55){ 
                if(shift1Active) SmartDashboard.putString("DriverStation/TimeToSwitch", "OFF IN " + (matchTimeInt - 55) + "s");
                else SmartDashboard.putString("DriverStation/TimeToSwitch", "ACTIVE IN " + (matchTimeInt - 55) + "s");
        }
            else if (matchTime > 30) {
                if(!shift1Active) SmartDashboard.putString("DriverStation/TimeToSwitch", "HUB ACTIVE");
                else SmartDashboard.putString("DriverStation/TimeToSwitch", "ACTIVE IN " + (matchTimeInt - 30) + "s");
            }
            else SmartDashboard.putString("DriverStation/TimeToSwitch", "HUB ACTIVE");
    }

    public void autoInit() {
        autoCommand = autoChooser.getSelected();
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
        //autoCommand.cancel();
        CommandScheduler.getInstance().cancelAll();
        shooterSubsystem.setSpeed(() -> 0.0);
        indexerSubsystem.setVoltage(() -> 0.0);
        intakeSubsystem.setDeployVoltage(() -> 0.0);
        intakeSubsystem.setRollerVoltage(() -> 0.0);
        matchTime.reset();
        matchTime.start();
    }
}
