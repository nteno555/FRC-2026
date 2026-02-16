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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.DriveCommands;
import frc2025.commands.SuperstructureCommands;
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

public class RobotContainer {
    public static final double DEAD_BAND_DRIVER = 0.1;
    public static final double DEAD_BAND_OPERATOR = 0.1;
    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);
    private final CommandXboxController testSystemsController = new CommandXboxController(2);
    private final CommandXboxController tuningController = new CommandXboxController(3);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    IndexerSubsystem indexerSubsystem;
    IntakeSubsystem intakeSubsystem;
    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    Autos autos;
    Command autoCommand;

    private double driveSpeed = 0.5;
    private double rotationSpeed = 0.7;

    public RobotContainer() {
        createSubsystems();
        configureBindings();
        configureAutoCommands();
//        autoCommand = autos.MidStationDepotShootWhileMoving();
        //autoCommand = autos.MidDepotNeutralClimbShootWhileMoving();
//        autoCommand = autos.MidDepotShootWhileMoving();
//        configureAutoCommands();
        //autoCommand = autos.LowShootPreload();
//        autoCommand = Autos.LowStationDepotClimb;
    }

    public void periodic() {
    }

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

    public void configureAutoCommands(){
        autos = new Autos(this);
        autoChooser.setDefaultOption("MidStationDepotClimb", autos.MidStationDepotClimb());

        autoChooser.addOption("Low Neutral (no steal)", autos.LowNeutralLowNeutralMidLow());
        autoChooser.addOption("Low Neutral (steal)", autos.LowStealLowNeutralLow());
        autoChooser.addOption("Mid Station Depot", autos.MidStationDepotClimb());
        autoChooser.addOption("High Neutral (no steal)", autos.HighNeutralHighNeutralMid());
        autoChooser.addOption("High Neutral (steal)", autos.HighStealHighNeutralClimb());
//        autoChooser.addOption("HighShootIntakeClimb", autos.HighShootIntakeClimb());
//        autoChooser.addOption("LowShuttleAuto", autos.LowShuttleAuto());
//        autoChooser.addOption("Taxi", autos.Taxi());

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
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveTeleop(
                        () -> (-driveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (-driveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (rotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                )
        );

        intakeSubsystem.setDefaultCommand(intakeSubsystem.setDeployVoltage(() -> 0.0));
        hoodSubsystem.setDefaultCommand(hoodSubsystem.autoAim());
        shooterSubsystem.setDefaultCommand(shooterSubsystem.setVoltage(() -> 0.0));
        indexerSubsystem.setDefaultCommand(indexerSubsystem.setVoltage(() -> 0.0, () -> 0.0));
//        turretSubsystem.setDefaultCommand(turretSubsystem.autoAim());

//        driverXboxController.a().onTrue(new InstantCommand(() -> {
//            if(!Alliance.get()){
//                driveSubsystem.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
//            } else {
//                driveSubsystem.resetOdometry(new Pose2d());
//            }
//        }));

        driverXboxController.a().onTrue(
                SuperstructureCommands.resetOdometry(
                        this,
                        () -> !Alliance.get()
                                ? new Pose2d(new Translation2d(), new Rotation2d(Math.PI))
                                : new Pose2d()
                )
        );

        driverXboxController.b().onTrue(
                DriveCommands.autoTrench(this)
        );

        driverXboxController
                .x()
                .and(driverXboxController.back().negate())
                .onTrue(
                hoodSubsystem.autoAimFlush()
                        .alongWith(shooterSubsystem.autoAimFlush())
                        .alongWith(turretSubsystem.setPosition(() -> 0.0)));

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
                .whileTrue(turretSubsystem.rezeroCRT());

        driverXboxController.povLeft() // left bottom paddle
                .onTrue(intakeSubsystem.setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage));
        driverXboxController.povDown() // right bottom paddle
                .onTrue(intakeSubsystem.setRollerVoltage(() -> 0.0));

        driverXboxController.leftTrigger()
                .whileTrue(
                        hoodSubsystem.autoAim()
                                .alongWith(shooterSubsystem.autoAim())
                                .alongWith(AutoAimCommands.driveAutoAim(this, () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                        () -> driveSubsystem.getTeleopSpeeds(
                                                (-0.5) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                                (-0.5) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                                (0.7) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                                        )))
                                .alongWith(AutoAimCommands.autoIndex(this)));

//        if(SubsystemConstants.hasTurret) {
//            driverXboxController.leftTrigger()
//                    .whileTrue(shooterSubsystem.autoAim()
//                            .alongWith(turretSubsystem.autoAim())
//                            .alongWith(hoodSubsystem.autoAim())
//                            .alongWith(AutoAimCommands.autoIndex(this)))
//                    .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));
//        } else {
//            driverXboxController.leftTrigger()
//                    .whileTrue(shooterSubsystem.autoAim()
//                            .alongWith(AutoAimCommands.driveAutoAim(
//                                    this,
//                                    () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
//                                    () -> driveSubsystem.getTeleopSpeeds(
//                                            (-0.5) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                                            (-0.5) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
//                                            (0.7) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
//                                    )
//                            ))
//                            .alongWith(hoodSubsystem.autoAim())
//                            .alongWith(AutoAimCommands.autoIndex(this)))
//                    .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));
//        }

        driverXboxController.povUp() // left middle paddle
                .onTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition));
        driverXboxController.povRight() // right middle paddle
                .onTrue(intakeSubsystem.setDeployPosition(() -> 0.0));

//        driverXboxController.rightTrigger()
//                .whileTrue(indexerSubsystem.setVoltage(() -> SubsystemConstants.spindexerShootVoltage,
//                        () -> SubsystemConstants.towerShootVoltage))
//                .whileFalse(indexerSubsystem.setVoltage(() -> 0.0, () -> 0.0));

        driverXboxController.leftBumper()
            .onTrue(Commands.runOnce(() -> {
                driveSpeed = 0.5;
                rotationSpeed = 0.7;
            }));

        driverXboxController.rightBumper()
                .onTrue(Commands.runOnce(
                        () -> {
                            driveSpeed = 0.3;
                            rotationSpeed = 0.5;
                        }));

        operatorXboxController.leftBumper()
                .onTrue(hoodSubsystem.autoAimFlush()
                        .alongWith(shooterSubsystem.autoAimFlush())
                        .alongWith(turretSubsystem.setPosition(() -> 0.0))
                );
        operatorXboxController.rightBumper()
                .onTrue(
                        hoodSubsystem.autoAim()
                                .alongWith(shooterSubsystem.autoAim())
                                .alongWith(turretSubsystem.autoAim())
                );
        operatorXboxController.y()
                .onTrue(Commands.runOnce(() -> RobotState.getInstance().setWantAutoShoot(!RobotState.getInstance().getWantAutoShoot())));
//        operatorXboxController.rightTrigger()
//                .whileTrue(indexerSubsystem.setVoltage(() -> -6.0, () -> -12.0));


//        tuningController.a()
//                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
//        tuningController.b()
//                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kForward));
//        tuningController.x()
//                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kForward));
//        tuningController.y()
//                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kForward));
//        tuningController.povLeft()
//                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));
//        tuningController.povUp()
//                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kReverse));
//        tuningController.povDown()
//                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kReverse));
//        tuningController.povRight()
//                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kReverse));
//        tuningController.leftBumper()
//                .whileTrue(driveSubsystem.runDriveSlipTest());
//        tuningController.rightBumper()
//                .whileTrue(new WheelRadiusCharacterizationCommand(driveSubsystem));

    tuningController.leftBumper().whileTrue(
            turretSubsystem.rezeroCRT()
    );

        tuningController.povLeft().whileTrue(
                turretSubsystem.setVoltage(() -> 2.5)
        ).whileFalse(turretSubsystem.setVoltage(() -> 0.0));

        tuningController.povRight().whileTrue(
                turretSubsystem.setVoltage(() -> -2.5)
        ).whileFalse(turretSubsystem.setVoltage(() -> 0.0));

        tuningController.povUp().whileTrue(
                turretSubsystem.setPositionSafe(() -> Math.PI)
        ).whileFalse(turretSubsystem.setVoltage(() -> 0.0));

        tuningController.povDown().whileTrue(
                turretSubsystem.setPositionSafe(() -> Math.PI*2.0)
        ).whileFalse(turretSubsystem.setVoltage(() -> 0.0));

//        tuningController.leftBumper().whileTrue(
//                turretSubsystem.autoAim()
//        );

//        tuningController.a().onTrue(new InstantCommand(() -> {
//            if(!Alliance.get()){
//                driveSubsystem.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
//            } else {
//                driveSubsystem.resetOdometry(new Pose2d());
//            }
//        }));

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
                                .alongWith()
//                                .alongWith(turretSubsystem.autoAim())
                )
                .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

        if(SubsystemConstants.hasTurret) {
            tuningController.rightBumper()
                    .whileTrue(shooterSubsystem.autoAim()
                            .alongWith(turretSubsystem.autoAim())
                            .alongWith(hoodSubsystem.autoAim())
                            .alongWith(AutoAimCommands.autoIndex(this)))
                    .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));
        } else {
            tuningController.rightBumper()
                    .whileTrue(shooterSubsystem.setSpeed(() -> SmartDashboard.getNumber("Tuning/RPM", 0.0))
                            .alongWith(AutoAimCommands.driveAutoAim(
                                    this,
                                    () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                                    () -> driveSubsystem.getTeleopSpeeds(
                                            (-0.5) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                            (-0.5) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                            (0.7) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                                    )
                            ))
                            .alongWith(hoodSubsystem.setPosition(() -> SmartDashboard.getNumber("Tuning/hoodPosition", 0.0))))
                    .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));
        }

        tuningController.rightTrigger().whileTrue(
            indexerSubsystem.setVoltage(() -> -6.0, () -> -12.0))
            .whileFalse(indexerSubsystem.setVoltage(() -> 0.0, () -> 0.0));

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
//        autoCommand = autos.HighDepotNeutralClimb();
//        autoCommand = autos.MidStationDepotForward();
        RobotState.getInstance().setShuttling(false);
        CommandScheduler.getInstance().schedule(autoCommand);
    }

    public void teleopInit() {
        //autoCommand.cancel();
        CommandScheduler.getInstance().cancelAll();
        RobotState.getInstance().setShuttling(false);
        shooterSubsystem.setSpeed(() -> 0.0);
        indexerSubsystem.setVoltage(() -> 0.0, () -> 0.0);
        intakeSubsystem.setDeployVoltage(() -> 0.0);
        intakeSubsystem.setRollerVoltage(() -> 0.0);
    }
}
