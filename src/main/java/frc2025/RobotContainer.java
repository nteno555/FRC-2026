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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.climber.ClimberIOReal;
import frc2025.subsystems.climber.ClimberIOSim;
import frc2025.subsystems.climber.ClimberSubsystem;
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

public class RobotContainer {
    public static final double DEAD_BAND_DRIVER = 0.1;
    public static final double DEAD_BAND_OPERATOR = 0.1;
    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);
    private final CommandXboxController testSystemsController = new CommandXboxController(2);
    private final CommandXboxController tuningController = new CommandXboxController(3);
    private final CommandXboxController driveTuningController = new CommandXboxController(4);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    ClimberSubsystem climberSubsystem;
    IndexerSubsystem indexerSubsystem;
    IntakeSubsystem intakeSubsystem;
    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    Autos autos;
    Command autoCommand;

    private double driveSpeed = 0.5;
    private double slowmodeDriveSpeed = 0.3;
    private double rotationSpeed = 0.7;
    private double slowmodeRotationSpeed = 0.5;
    private boolean slowmode;
    private boolean sysCheck;

    private AimMode aimMode;

    private enum AimMode {
        TURRET,
        FIXED,
        MANUAL
    }

    public RobotContainer() {
        createSubsystems();
        configureBindings();
        configureAutoCommands();

        if (SubsystemConstants.hasTurret) {
            aimMode = AimMode.TURRET;
        }
        else {
            aimMode = AimMode.FIXED;
        }
    }

    public void periodic() {
    }
//shooterSubststem.setDefaultCommand( just shoot me);
    // - Yuna 2026
    private void createSubsystems() {
        if(RobotBase.isReal()) {
            driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOReal());
            climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOReal());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());
            hoodSubsystem = new HoodSubsystem(new HoodIOReal());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOReal());
            turretSubsystem = new TurretSubsystem(new TurretIOReal());
        } else {
            driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOSim());
            climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
            hoodSubsystem = new HoodSubsystem(new HoodIOSim());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
            turretSubsystem = new TurretSubsystem(new TurretIOSim());
        }
    }

    public void configureAutoCommands() {
        autos = new Autos(this);
        autoChooser.setDefaultOption("Low Loop Neutral x2", autos.LowStealLowLoopLow());

//        autoChooser.setDefaultOption("High Neutral Depot", autos.HighNeutralDeepDepot());

        autoChooser.addOption("Low Deep Neutral x2", autos.LowNeutralLowNeutralLow());
        autoChooser.addOption("Low Steal Neutral x2", autos.LowStealLowNeutralLow());
        autoChooser.addOption("Low Steal Neutral x2 Station", autos.LowStealLowNeutralStation());
        autoChooser.addOption("Low Neutral Deep Station", autos.LowNeutralDeepStation());
        autoChooser.addOption("Low Loop Neutral x2", autos.LowStealLowLoopLow());

        autoChooser.addOption("Mid Station Depot", autos.MidStationDepotClimb());

        autoChooser.addOption("High Deep Neutral x2", autos.HighNeutralHighNeutralHigh());
        autoChooser.addOption("High Steal Neutral x2", autos.HighStealHighNeutralHigh());
        autoChooser.addOption("High Loop Neutral x2", autos.HighStealHighLoopHigh());
        autoChooser.addOption("High Steal Neutral x2 (start under trench)", autos.HighStealHighLoopHigh_StartUnderTrench());

        autoChooser.addOption("Olivia Auto", autos.OliviasSuperAwesomeAuto());
//        autoChooser.addOption("Low Shuttle", autos.LowShuttleAuto());

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

        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveTeleop(
                        () -> (driveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY() + testSystemsController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (driveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX() + testSystemsController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX() + testSystemsController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                )
        );

        intakeSubsystem.setDefaultCommand(intakeSubsystem.setDeployVoltage(() -> 0.0));
        hoodSubsystem.setDefaultCommand(hoodSubsystem.autoAim());
        shooterSubsystem.setDefaultCommand(shooterSubsystem.setVoltage(() -> 0.0));
        indexerSubsystem.setDefaultCommand(AutoAimCommands.autoIndex(this));
        turretSubsystem.setDefaultCommand(turretSubsystem.setVoltage(() -> 0.0));
//        turretSubsystem.setDefaultCommand(turretSubsystem.autoAim());

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
                                .and(() -> aimMode == AimMode.TURRET)
                                .onTrue(
                                        turretSubsystem.setVoltage(() -> 0.0)
                                                .alongWith(
                                                        shooterSubsystem.autoAim().withTimeout(1)
                                                        .andThen(shooterSubsystem.setVoltage(() -> 0.0))
                                                )
                                );

        driverXboxController
                .b()
                        .and(driverXboxController.back().negate())
                        .and(() -> (aimMode == AimMode.FIXED))
                        .onTrue(
                                shooterSubsystem.autoAim().withTimeout(1)
                                        .andThen(shooterSubsystem.setVoltage(() -> 0.0))
                        );

        driverXboxController
                .b()
                        .and(driverXboxController.back().negate())
                        .and(() -> aimMode == AimMode.MANUAL)
                        .onTrue(
                                shooterSubsystem.setSpeed(() -> ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(1.12))
                                        .withTimeout(1)
                                        .andThen(
                                                shooterSubsystem.setVoltage(() -> 0.0)
                                        )
                                );

        driverXboxController
                .x()
                        .and(driverXboxController.back().negate())
                        .and(() -> (aimMode == AimMode.FIXED))
                                .onTrue(
                                        AutoAimCommands.driveAutoAim(
                                                this,
                                                        () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                                        () -> driveSubsystem.getTeleopSpeeds(
                                                                (driveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                                                (driveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                                                (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                                                        ))
                                        .alongWith(shooterSubsystem.autoAim())
                                        .alongWith(intakeSubsystem.oscillateIntake())
                                );

        driverXboxController
                .x()
                        .and(driverXboxController.back().negate())
                        .and(() -> (aimMode == AimMode.TURRET))
                                .onTrue(shooterSubsystem.autoAim()
                                        .alongWith(turretSubsystem.autoAim()));

        driverXboxController
                .x()
                        .and(driverXboxController.back().negate())
                                .and(() -> (aimMode == AimMode.MANUAL))
                                .onTrue(shooterSubsystem.setSpeed(() -> ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(1.12))
                                        .alongWith(indexerSubsystem.setVoltage(() -> SubsystemConstants.indexerShootVoltage))
                                        .alongWith(intakeSubsystem.oscillateIntake())
                                );

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
                .whileTrue(turretSubsystem.setPositionSafe(() -> 0.0, () -> 0.0));

        driverXboxController.povLeft() // left bottom paddle
                .onTrue(intakeSubsystem.setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage));
        driverXboxController.povDown() // right bottom paddle
                .onTrue(intakeSubsystem.setRollerVoltage(() -> 0.0));

//        driverXboxController.leftTrigger()
//                .and(() -> (aimMode == AimMode.FIXED))
//                .whileTrue(
//                        AutoAimCommands.driveAutoAim(this,
//                            () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
//                            () -> driveSubsystem.getTeleopSpeeds(
//                                    (driveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                                    (driveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                                    (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
//                            )
//                        )
//                                .alongWith(shooterSubsystem.autoAim())
//                                .alongWith(hoodSubsystem.autoAim())
//                        .alongWith(intakeSubsystem.oscillateIntake())
//                )
//                .onFalse(
//                        shooterSubsystem.autoAim().withTimeout(1)
//        );
//
//        driverXboxController.leftTrigger()
//                        .and(() -> aimMode == AimMode.TURRET)
//                                .whileTrue(
//                                        AutoAimCommands.turretAutoAim(this)
//                                                .alongWith(intakeSubsystem.oscillateIntake())
//                                )
//                .onFalse(shooterSubsystem.autoAim().withTimeout(1));
//
//        driverXboxController.leftTrigger()
//                .and(() -> (aimMode == AimMode.MANUAL))
//                        .whileTrue(shooterSubsystem.setSpeed(() -> ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(1.12))
//                                .alongWith(indexerSubsystem.setVoltage(() -> SubsystemConstants.indexerShootVoltage))
//                                .alongWith(intakeSubsystem.oscillateIntake())
//                        )
//                        .onFalse(
//                                shooterSubsystem.setSpeed(() -> ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(1.12)
//                                ).withTimeout(1)
//                        );

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

        operatorXboxController.x()
                .and(() -> (aimMode == AimMode.MANUAL))
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().setManualShuttle(!RobotState.getInstance().getManualShuttle())));

        operatorXboxController.a()
                .whileTrue(intakeSubsystem.oscillateIntake())
                .onFalse(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition));

        new Trigger(() -> (aimMode == AimMode.MANUAL))
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().setManualShuttleOverride(true)))
                .whileTrue(turretSubsystem.setVoltage(() -> 3.0 * operatorXboxController.getRightX()));

        new Trigger(() -> (aimMode != AimMode.MANUAL))
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().setManualShuttleOverride(false)));

        new Trigger(() -> (aimMode == AimMode.FIXED))
                .whileTrue(turretSubsystem.setVoltage(() -> 0.0));

        new Trigger(() -> slowmode).whileTrue(
                driveSubsystem.driveTeleop(
                        () -> (slowmodeDriveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (slowmodeDriveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (-slowmodeRotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                )
                        .alongWith(intakeSubsystem.oscillateIntake())
        );

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


    tuningController.leftBumper().whileTrue(
            turretSubsystem.rezeroCRT()
    );

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
//    tuningController.leftBumper().whileTrue(
//            turretSubsystem.autoAim()
//    );


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

    public double getDriverLeftY() {
        return driverXboxController.getLeftY();
    }

    public double getDriverLeftX() {
        return driverXboxController.getLeftX();
    }

    public void autoInit() {
        autoCommand = autoChooser.getSelected();
//        if(!Utils.isSimulation()) {
//        } else {
////            autoCommand = autos.LowStealLowNeutralStation();
//            autoCommand = autos.HighStealHighNeutralClimb();
//        }
//        autoCommand = autos.HighDepotNeutralClimb();
//        autoCommand = autos.MidStationDepotForward();
        CommandScheduler.getInstance().schedule(autoCommand);
    }

    public void teleopInit() {
        //autoCommand.cancel();
        CommandScheduler.getInstance().cancelAll();
        shooterSubsystem.setSpeed(() -> 0.0);
        indexerSubsystem.setVoltage(() -> 0.0);
        intakeSubsystem.setDeployVoltage(() -> 0.0);
        intakeSubsystem.setRollerVoltage(() -> 0.0);
    }
}
