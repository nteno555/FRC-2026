package frc2025;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc2025.subsystems.drive.DriveConstants.BackLeft;
import static frc2025.subsystems.drive.DriveConstants.BackRight;
import static frc2025.subsystems.drive.DriveConstants.DrivetrainConstants;
import static frc2025.subsystems.drive.DriveConstants.FrontLeft;
import static frc2025.subsystems.drive.DriveConstants.FrontRight;
import static frc2025.subsystems.drive.DriveConstants.kSpeedAt12Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.DriveToPoseCommand;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.SubsystemConstants;

import frc2025.subsystems.drive.DriveIOReal;
import frc2025.subsystems.drive.DriveIOSim;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.hood.HoodIOReal;
import frc2025.subsystems.hood.HoodIOSim;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.subsystems.indexer.IndexerIOReal;
import frc2025.subsystems.indexer.IndexerIOSim;
import frc2025.subsystems.indexer.IndexerSubsystem;
import frc2025.subsystems.intake.IntakeIOReal;
import frc2025.subsystems.intake.IntakeIOSim;
import frc2025.subsystems.intake.IntakeSubsystem;
import frc2025.subsystems.shooter.ShooterIOReal;
import frc2025.subsystems.shooter.ShooterIOSim;
import frc2025.subsystems.shooter.ShooterSubsystem;
import frc2025.subsystems.vision.VisionIOReal;
import frc2025.subsystems.vision.VisionIOSim;
import frc2025.subsystems.vision.VisionSubsystem;

import frc2025.util.Alliance;
import java.util.ArrayList;

public class RobotContainer {
    public static final double DEAD_BAND_DRIVER = 0.1;
    public static final double DEAD_BAND_OPERATOR = 0.1;
    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);
    private final CommandXboxController testSystemsController = new CommandXboxController(2);
    private final CommandXboxController treeMapTuningController = new CommandXboxController(5);
    private final CommandXboxController tuningController = new CommandXboxController(3);
    private final CommandXboxController driveTuningController = new CommandXboxController(4);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    ShooterSubsystem shooterSubsystem;
    HoodSubsystem hoodSubsystem;
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;

    Autos autos;
    Command autoCommand;

    private double driveSpeed = 0.8;
    private double rotationSpeed = 0.7;

    private boolean aiming = false;

    public RobotContainer() {
        createSubsystems();
        configureBindings();
        loggerInit();
    }

    public void periodic() {
        loggerPeriodic();
    }

    private void createSubsystems() {
        if(RobotBase.isReal()) {
            driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOReal());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOReal());
            hoodSubsystem = new HoodSubsystem(new HoodIOReal());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOReal());

        } else {
            driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOSim());
            shooterSubsystem = new ShooterSubsystem(new ShooterIOSim());
            hoodSubsystem = new HoodSubsystem(new HoodIOSim());
            intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
            indexerSubsystem = new IndexerSubsystem(new IndexerIOSim());
        }
    }

    public DriveSubsystem getDrive() { return driveSubsystem; }

    public VisionSubsystem getVision() { return visionSubsystem; }

    public ShooterSubsystem getShooter() { return shooterSubsystem; }

    public HoodSubsystem getHood() { return hoodSubsystem; }

    public IntakeSubsystem getIntake() { return intakeSubsystem; }
    
    public IndexerSubsystem getIndexer() { return indexerSubsystem; }
    

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveTeleop(
                        () -> (driveSpeed) * handleDeadBand(driverXboxController.getLeftY() + tuningController.getLeftY() + treeMapTuningController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (driveSpeed) * handleDeadBand(driverXboxController.getLeftX() + tuningController.getLeftX() + treeMapTuningController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                        () -> (-rotationSpeed) * handleDeadBand(driverXboxController.getRightX() + tuningController.getRightX() + treeMapTuningController.getRightX(), DEAD_BAND_DRIVER, true) * Math.PI * 2
                )
        );




        //--------------------DRIVER XBOX CONTROLLER------------
        driverXboxController.a().onTrue(
                SuperstructureCommands.resetOdometry(
                        this,
                        () -> !Alliance.get()
                                ? new Pose2d(new Translation2d(), new Rotation2d(Math.PI))
                                : new Pose2d()
                )
        );

        new Trigger(() -> aiming)
                    .whileTrue(shooterSubsystem.setSpeed(() -> SubsystemConstants.flushWheelRps)
                                    .alongWith(hoodSubsystem.setPosition(() -> SubsystemConstants.flushHoodRotations)));
        
        new Trigger(() -> true)
                    .whileTrue(AutoAimCommands.autoIndex(this));
        
        driverXboxController.y()
                    .onTrue(Commands.runOnce(() -> aiming = true));

        driverXboxController.x()
                    .onTrue(Commands.runOnce(() -> aiming = false));
        
        driverXboxController.leftTrigger()
                    .onTrue(driveSubsystem.driveAutoAim());

        driverXboxController.povDown()
                    .onTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition));
        
        driverXboxController.povUp()
                    .onTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeRetractPosition));

        driverXboxController.leftBumper()
                    .onTrue(intakeSubsystem.setRollerVoltage(() -> SubsystemConstants.intakeRollerVoltage));

        driverXboxController.rightBumper()
                    .onTrue(intakeSubsystem.setRollerVoltage(() -> 0.0));

        // ---------- TUNING CONTROLLER----------------
        SmartDashboard.putNumber("Tuning/intakeDeployPosition", 0.0);
        SmartDashboard.putNumber("Tuning/shooterRps", 0.0);
        SmartDashboard.putNumber("Tuning/hoodPosition", 0.0);
        SmartDashboard.putNumber("Tuning/hoodVoltageStepkV", 0.0);

        SmartDashboard.putNumber("Tuning/IntakeRollerVoltage", 0.0);
        SmartDashboard.putNumber("Tuning/ShooterVoltage", 0.0);
        SmartDashboard.putNumber("Tuning/LindexerVoltage", 0.0);
        SmartDashboard.putNumber("Tuning/ShooterkV", 0.0);
        SmartDashboard.putNumber("Tuning/HoodkV", 0.0);

        SmartDashboard.putNumber("Tuning/IntakeRollerVoltage", 0.0);
        SmartDashboard.putNumber("Tuning/IntakeDeployVoltage", 0.0);
        SmartDashboard.putNumber("Tuning/ShooterkA", 0.0);

        tuningController.rightTrigger()
                        .whileTrue(hoodSubsystem.setPosition(() -> SmartDashboard.getNumber("Tuning/hoodPosition", 0.0)))
                        .whileFalse(hoodSubsystem.setVoltage(() -> 0.0));
        
        tuningController.a()
                        .onTrue(hoodSubsystem.rezero());
                        

        tuningController.leftTrigger()
                        .whileTrue(shooterSubsystem.setSpeed(() -> 30.0))
                        .whileFalse(shooterSubsystem.setVoltage(() -> 0.0));

        //    tuningController.leftTrigger()
        //                    .whileTrue(shooterSubsystem.setSpeed(() -> SmartDashboard.getNumber("Tuning/shooterRps", 0.0)));
//
        //    tuningController.leftBumper()
        //                    .whileTrue(hoodSubsystem.characterizeVoltageStep(() -> SmartDashboard.getNumber("Tuning/hoodVoltageStepkV", 0.0)));

        // tuningController.rightTrigger()
        //                 .whileTrue(hoodSubsystem.characterizeVoltageStep(() -> SmartDashboard.getNumber("Tuning/HoodkV", 0.0)));

        tuningController.rightBumper()
                .whileTrue(intakeSubsystem.setRollerVoltage(() -> SmartDashboard.getNumber("Tuning/IntakeRollerVoltage", 0.0)))
                .whileFalse(intakeSubsystem.setRollerVoltage(() -> 0.0));

        tuningController.leftBumper()
                .whileTrue(intakeSubsystem.setDeployPosition(() -> -SmartDashboard.getNumber("Tuning/intakeDeployPosition", 0.0)));

        // tuningController.leftTrigger()
        //                 .whileTrue(hoodSubsystem.rezero());

        tuningController.povDown()
                .whileTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeRetractPosition))
                .whileFalse(intakeSubsystem.setDeployVoltage(() -> SubsystemConstants.intakeDeployPosition));

        tuningController.povUp()
                .whileTrue(intakeSubsystem.characterizeStaticVoltage());

//        tuningController.povDown()
//                .onTrue(intakeSubsystem.setDeployVoltage(() -> SmartDashboard.getNumber("Tuning/IntakeDeployVoltage", 0)));
//
//        tuningController.povUp()
//                .onTrue(intakeSubsystem.setRollerVoltage(() -> SmartDashboard.getNumber("Tuning/IntakeRollerVoltage", 0)));

        // tuningController.rightBumper()
        //                 .whileTrue(intakeSubsystem.setRollerVoltage(() -> SmartDashboard.getNumber("Tuning/IntakeRollerVoltage", 0.0)))
        //                 .whileFalse(intakeSubsystem.setRollerVoltage(() -> 0.0));

        // tuningController.b()
        //             .whileTrue(hoodSubsystem.setPosition(() -> SmartDashboard.getNumber("Tuning/hoodPosition", 0.0)));

        tuningController.y()
                .whileTrue(indexerSubsystem.setVoltage(() -> SmartDashboard.getNumber("Tuning/LindexerVoltage", 0.0)))
                .whileFalse(indexerSubsystem.setVoltage(() -> 0.0));

        // tuningController.povUp()
        //             .whileTrue(shooterSubsystem.characterizeStaticVoltage());

//        tuningController.povDown()
//                    .whileTrue(shooterSubsystem.characterizeVoltageStep(() -> SmartDashboard.getNumber("Tuning/ShooterkV", 0.0)));


        // ----------- TREE MAP TUNING CONTROLLER ---------------------
        treeMapTuningController.a().onTrue(
                SuperstructureCommands.resetOdometry(
                        this,
                        () -> !Alliance.get()
                                ? new Pose2d(new Translation2d(), new Rotation2d(Math.PI))
                                : new Pose2d()
                )
        );

        treeMapTuningController.leftTrigger()
                        .whileTrue(shooterSubsystem.setSpeed(()-> SmartDashboard.getNumber("Tuning/ShooterTreeMapRPS", 0.0)))
                        .whileFalse(shooterSubsystem.setVoltage(()->0.0));
        
        
        treeMapTuningController.leftBumper()
                        .whileTrue(hoodSubsystem.setPosition(()-> SmartDashboard.getNumber("Tuning/ShooterTreeMapHoodAngle", 0.0)))
                        .whileFalse(hoodSubsystem.setVoltage(()->0.0));
        
        treeMapTuningController.rightTrigger()
                        .whileTrue(driveSubsystem.driveAutoAim());
        
        treeMapTuningController.rightBumper()
                        .whileTrue(
                            indexerSubsystem.setVoltage(() -> -7.0)
                            .withTimeout(5.0)
                            .andThen(indexerSubsystem.setVoltage(()->7.0)))//negative is forward
                        .whileFalse(indexerSubsystem.setVoltage(() -> 0.0));
        
        treeMapTuningController.povUp()
                        .onTrue(intakeSubsystem.setRollerVoltage(() -> 4.0));
        
        treeMapTuningController.povDown()
                        .onTrue(intakeSubsystem.setRollerVoltage(() -> 0.0));
        
        treeMapTuningController.povLeft()
                        .onTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeRetractPosition));

        treeMapTuningController.povRight()
                        .onTrue(intakeSubsystem.setDeployPosition(() -> SubsystemConstants.intakeDeployPosition));

        SmartDashboard.putNumber("Tuning/ShooterTreeMapRPS", 0.0);
        SmartDashboard.putNumber("Tuning/ShooterTreeMapHoodAngle",0.0);

        //------------------- DRIVE TUNING CONTROLLER -----------------------

        driveTuningController.a()
                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kForward));
        driveTuningController.b()
                .whileTrue(driveSubsystem.runDriveDynamicTest(SysIdRoutine.Direction.kReverse));
        driveTuningController.x()
                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kForward));
        driveTuningController.y()
                .whileTrue(driveSubsystem.runDriveQuasiTest(SysIdRoutine.Direction.kReverse));
        driveTuningController.povLeft()
                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kForward));
        driveTuningController.povUp()
                .whileTrue(driveSubsystem.runSteerDynamicTest(SysIdRoutine.Direction.kReverse));
        driveTuningController.povDown()
                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kForward));
        driveTuningController.povRight()
                .whileTrue(driveSubsystem.runSteerQuasiTest(SysIdRoutine.Direction.kReverse));

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

    public void loggerInit(){

    }

    public void loggerPeriodic(){
        

    }

    public void autoInit() {
        CommandScheduler.getInstance().schedule(autoCommand);
    }

    public void teleopInit() {
        //autoCommand.cancel();
        CommandScheduler.getInstance().cancelAll();
    }
}