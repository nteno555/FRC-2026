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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2025.commands.AutoAimCommands;
import frc2025.config.ShooterTreeMap;
import frc2025.subsystems.drive.DriveIOReal;
import frc2025.subsystems.drive.DriveIOSim;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.indexer.IndexerIOReal;
import frc2025.subsystems.indexer.IndexerIOSim;
import frc2025.subsystems.indexer.IndexerSubsystem;
import frc2025.subsystems.intake.IntakeIOReal;
import frc2025.subsystems.intake.IntakeIOSim;
import frc2025.subsystems.intake.IntakeSubsystem;
import frc2025.subsystems.pivot.PivotIOReal;
import frc2025.subsystems.pivot.PivotIOSim;
import frc2025.subsystems.pivot.PivotSubsystem;
import frc2025.subsystems.shooter.ShooterIOReal;
import frc2025.subsystems.shooter.ShooterIOSim;
import frc2025.subsystems.shooter.ShooterSubsystem;
import frc2025.subsystems.vision.VisionIOReal;
import frc2025.subsystems.vision.VisionIOSim;
import frc2025.subsystems.vision.VisionSubsystem;
import frc2025.util.Alliance;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc2025.subsystems.drive.DriveConstants.*;

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
    PivotSubsystem pivotSubsystem;
    ShooterSubsystem shooterSubsystem;

    Autos autos;
    Command autoCommand;
    private boolean tuning = false;
    private boolean wantIntake = false;

    public RobotContainer() {
        createSubsystems();
        configureBindings();
        autos = new Autos(this);
        autoCommand = autos.MidShootPreload();
        configureAutoCommands();
        //autoCommand = autos.LowShootPreload();
    }

    public void periodic() {
    }

    private void createSubsystems() {
        if(RobotBase.isReal()) {
            driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOReal());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOReal());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());
            pivotSubsystem = new PivotSubsystem(new PivotIOReal());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOReal());
        } else {
            driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOSim());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
            pivotSubsystem = new PivotSubsystem(new PivotIOSim());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
        }
    }

    public void configureAutoCommands(){
        autoChooser.setDefaultOption("Do nothing", null);

        autoChooser.addOption("LowShootPreload", autos.LowShootPreload());
        autoChooser.addOption("MidShootPreload", autos.MidShootPreload());
        autoChooser.addOption("HighShootClimb", autos.HighShootClimb());
        autoChooser.addOption("HighShootIntakeClimb", autos.HighShootIntakeClimb());
        autoChooser.addOption("LowShuttleAuto", autos.LowShuttleAuto());
        autoChooser.addOption("Taxi", autos.Taxi());

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

    public PivotSubsystem getPivot() {
        return pivotSubsystem;
    }

    public ShooterSubsystem getShooter() {
        return shooterSubsystem;
    }

    public VisionSubsystem getVision() {
        return visionSubsystem;
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveTeleop(
                        () -> (0.3) * handleDeadBand(driverXboxController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (0.3) * handleDeadBand(driverXboxController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (0.3) * handleDeadBand(driverXboxController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                )
        );

        driverXboxController.a().onTrue(new InstantCommand(() -> {
            if(!Alliance.get()){
                driveSubsystem.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
            } else {
                driveSubsystem.resetOdometry(new Pose2d());
            }
        }));




        
        SmartDashboard.putNumber("Tuning/Distance", 0.0);
        SmartDashboard.putNumber("Tuning/RPM", 0.0);
        SmartDashboard.putNumber("Tuning/pivotPosition", 0.0);

        SmartDashboard.putData("Tuning/Add Point", new InstantCommand(() -> {
            double dist = SmartDashboard.getNumber("Tuning/Distance", 0.0);
            double rpm = SmartDashboard.getNumber("Tuning/RPM", 0.0);
            double pivotPos = SmartDashboard.getNumber("Tuning/pivotPosition", 0.0);

            ShooterTreeMap.getInstance().addShot(dist,rpm);
            //SmartDashboard.putString("Tuning/lowerWheelTree Map", ShooterTreeMap.getInstance().lowerWheelTreeMap.toString());

        }).ignoringDisable(true));

//        SmartDashboard.putNumber("Tuning/Remove Distance", 0.0);
//
//        SmartDashboard.putData("Tuning/Remove Point", new InstantCommand(() -> {
//            double dist = SmartDashboard.getNumber("Tuning/Remove Distance", 0.0);
//
//            ShooterTreeMap.getInstance().removeShot(dist);
//
//        }).ignoringDisable(true));

        //indexerSubsystem.setDefaultCommand(indexerSubsystem.setVoltage(() -> 0.0));
        //intakeSubsystem.setDefaultCommand(intakeSubsystem.setVoltage(() -> 0.0));

        driverXboxController.rightTrigger().whileTrue(
            indexerSubsystem.setVoltage(() -> -6.0, () -> -12.0))
            .whileFalse(indexerSubsystem.setVoltage(() -> 0.5, () -> 0.0));

        driverXboxController.povLeft().whileTrue(
                new InstantCommand(() -> wantIntake = !wantIntake)
                        .andThen(intakeSubsystem.setIntaking(() -> wantIntake))
        );


        driverXboxController.leftTrigger()
               .whileTrue(
                       AutoAimCommands.autoAimWhileMovingCommand(
                               this,
                               ShooterTreeMap.getInstance(),
                               () -> driveSubsystem.getTeleopSpeeds(
                                       (DriveSubsystem.slowmode ? 0.5 : 1.0) * handleDeadBand(driverXboxController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                       (DriveSubsystem.slowmode ? 0.5 : 1.0) * handleDeadBand(driverXboxController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                       (DriveSubsystem.slowmode ? 0.75 : 1.0) * handleDeadBand(driverXboxController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                               ))
               ).whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

               driverXboxController.povRight().whileTrue(pivotSubsystem.setPosition(() -> SmartDashboard.getNumber("Tuning/pivotPosition", 0.0)));


        driverXboxController.rightTrigger()
                .whileTrue(AutoAimCommands.autoIndex(this))
                .whileFalse(indexerSubsystem.setVoltage(() -> 0.0, () -> 0.0));

        driverXboxController.b().onTrue(new InstantCommand(() -> {
            RobotState.getInstance().setShuttling(!RobotState.getInstance().getShuttling());
        }));

        //testSystemsController.a()
         //               .whileTrue(inta)

        testSystemsController.leftBumper()
                        .whileTrue(shooterSubsystem.setSpeed(() -> 2400.0))
                                .whileFalse(shooterSubsystem.setSpeed(() -> 0.0));

        testSystemsController.leftTrigger()
                        .whileTrue(shooterSubsystem.setVoltage(() -> 6.0))
                                .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

        testSystemsController.rightBumper()
                        .whileTrue(indexerSubsystem.setVoltage(() -> -1.5, () -> -1.5))
                                .whileFalse(indexerSubsystem.setVoltage(() -> 0.0, () -> 0.0));

        testSystemsController.rightTrigger()
                        .whileTrue(intakeSubsystem.setVoltage(() -> 12.0))
                                .whileFalse(intakeSubsystem.setVoltage(() -> 0.0));

        tuningController.rightBumper()
                        .whileTrue(shooterSubsystem.setVoltage(() -> 3.0))
                .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

        tuningController.a()
                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
        tuningController.b()
                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kForward));
        tuningController.x()
                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kForward));
        tuningController.y()
                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kForward));
        tuningController.povLeft()
                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));
        tuningController.povUp()
                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kReverse));
        tuningController.povDown()
                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kReverse));
        tuningController.povRight()
                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kReverse));
        tuningController.leftBumper()
                .whileTrue(driveSubsystem.runDriveSlipTest());
//        tuningController.rightBumper()
//                .whileTrue(new WheelRadiusCharacterizationCommand(driveSubsystem));

        SmartDashboard.putNumber("Tuning/RPM", 0.0);
        tuningController.rightBumper()
                .whileTrue(shooterSubsystem.setSpeed(() -> SmartDashboard.getNumber("Tuning/RPM", 0.0)))
                .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

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
//        autoCommand = autoChooser.getSelected();
        autoCommand = autos.MidDepotShootWhileMoving();
        CommandScheduler.getInstance().schedule(autoCommand);
    }

    public void teleopInit() {
        //autoCommand.cancel();
        CommandScheduler.getInstance().cancel(autoCommand);
        RobotState.getInstance().setShuttling(false);
        shooterSubsystem.setVoltage(() -> 0.0);
        indexerSubsystem.setVoltage(() -> 0.0, () -> 0.0);
        intakeSubsystem.setVoltage(() -> 0.0);
    }
}
