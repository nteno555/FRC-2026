package frc2025;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc2025.commands.AutoScoreCommands;
import frc2025.commands.AutoScoreCommands;
import frc2025.commands.IntakeCommands;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.Superstructure;
import frc2025.subsystems.arm.ArmIOReal;
import frc2025.subsystems.arm.ArmIOSim;
import frc2025.subsystems.arm.ArmSubsystem;
import frc2025.subsystems.climber.ClimberIO;
import frc2025.subsystems.climber.ClimberIOReal;
import frc2025.subsystems.climber.ClimberIOSim;
import frc2025.subsystems.climber.ClimberSubsystem;
import frc2025.subsystems.drive.DriveConstants;
import frc2025.subsystems.drive.DriveIOReal;
import frc2025.subsystems.drive.DriveIOSim;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.elevator.ElevatorIO;
import frc2025.subsystems.elevator.ElevatorIOReal;
import frc2025.subsystems.elevator.ElevatorIOSim;
import frc2025.subsystems.elevator.ElevatorSubsystem;
import frc2025.subsystems.endEffector.EndEffectorIOReal;
import frc2025.subsystems.endEffector.EndEffectorIOSim;
import frc2025.subsystems.endEffector.EndEffectorSubsystem;
import frc2025.subsystems.funnel.FunnelIOSim;
import frc2025.subsystems.vision.VisionIOReal;
import frc2025.subsystems.vision.VisionIOSim;
import frc2025.subsystems.vision.VisionSubsystem;
import frc2025.subsystems.funnel.FunnelIOReal;
import frc2025.subsystems.funnel.FunnelSubsystem;
import frc2025.util.AlignPathBuilder;
import frc2025.util.Alliance;
import frc2025.util.Keyboard;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc2025.subsystems.drive.DriveConstants.*;

public class RobotContainer {
    /*private final XboxController operatorXboxController = new XboxController(1);
	private final XboxController driverXboxController = new XboxController(0);
	private final Keyboard operatorKeyboard = new Keyboard(2);*/
    public static final double DEAD_BAND_DRIVER = 0.1;
    public static final double DEAD_BAND_OPERATOR = 0.1;
    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);
    private final CommandXboxController testSystemsController = new CommandXboxController(2);
    private SlewRateLimiter xDriveStickLimiter = new SlewRateLimiter(3);
	private SlewRateLimiter yDriveStickLimiter = new SlewRateLimiter(3);
    private int wantedId = 0;
    private int wantedLevel = 4;
    private boolean wantScore = true;
    private Supplier<Integer> reefIdSupplier;

    private Map<Integer, Integer> reefKeyboardMap = new HashMap<>(
			Map.ofEntries(
					Map.entry(65, 1),
					Map.entry(64, 2),
					Map.entry(62, 3),
					Map.entry(61, 4),
					Map.entry(47, 5),
					Map.entry(32, 6),
					Map.entry(18, 7),
					Map.entry(19, 8),
					Map.entry(20, 9),
					Map.entry(21, 10),
					Map.entry(36, 11),
					Map.entry(51, 0)
			)
	);

    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;
    FunnelSubsystem funnelSubsystem;
    EndEffectorSubsystem endEffectorSubsystem;
    DriveSubsystem driveSubsystem;
    Superstructure superstructure;
    VisionSubsystem visionSubsystem;
    ClimberSubsystem climberSubsystem;
    Autos autos;
    Command autoCommand;
    private boolean tuning = false;

    public RobotContainer() {
        createSubsystems();
        configureBindings();
        AlignPathBuilder.initialize();
        autos = new Autos(driveSubsystem, superstructure);
        //autoCommand = autos.High4Piece();
        //autoCommand = autos.Low4Piece();
        autoCommand = autos.Mid1CoralThenAlgae();
    }

    public void periodic() {
        int y = 0;//operatorKeyboard.getCleanY();
        if(reefKeyboardMap.containsKey(y))
            wantedId = reefKeyboardMap.get(y);
        Logger.recordOutput("Drive/robotContainerReefId", wantedId);
    }

    private void createSubsystems() {
        if(RobotBase.isReal()) {
                driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
                armSubsystem = new ArmSubsystem(new ArmIOReal());
                endEffectorSubsystem = new EndEffectorSubsystem(new EndEffectorIOReal());
                funnelSubsystem = new FunnelSubsystem(new FunnelIOReal());
                visionSubsystem = new VisionSubsystem(new VisionIOReal());
                climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
         } else {
             driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
             elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
             endEffectorSubsystem = new EndEffectorSubsystem(new EndEffectorIOSim());
             armSubsystem = new ArmSubsystem(new ArmIOSim());
             funnelSubsystem = new FunnelSubsystem(new FunnelIOSim());
             visionSubsystem = new VisionSubsystem(new VisionIOSim());
            climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
         }
         superstructure = new Superstructure(elevatorSubsystem, armSubsystem, funnelSubsystem, endEffectorSubsystem, climberSubsystem);

    }
    private void configureBindings() {
        //DoubleSupplier limitedX = () -> xDriveStickLimiter.calculate(driverXboxController.getLeftX());
		//DoubleSupplier limitedY = () -> yDriveStickLimiter.calculate(driverXboxController.getLeftY());

        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveTeleop(
                        () -> {
                            return (DriveSubsystem.slowmode ? 0.5 : 1.0) * handleDeadBand(driverXboxController.getLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12VoltsMps;
                        },
                        () -> {
                            return (DriveSubsystem.slowmode ? 0.5 : 1.0) * handleDeadBand(driverXboxController.getLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12VoltsMps;
                        },
                        () -> {
                            return (DriveSubsystem.slowmode ? 0.75 : 1.0) * -handleDeadBand(driverXboxController.getRightX(), DEAD_BAND_DRIVER, true) * kMaxAngularVelocity;
                        }
                )
        );

        driverXboxController.a().onTrue(new InstantCommand(() -> {
            if(!Alliance.get()){
                driveSubsystem.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
            } else {
                driveSubsystem.resetOdometry(new Pose2d());
            }
        }));

        if(!tuning) {
            elevatorSubsystem.setDefaultCommand(elevatorSubsystem.setFixed());
            armSubsystem.setDefaultCommand(armSubsystem.setFixed());
        }

//        driverXboxController.b()
//                .whileTrue(superstructure.setWantedStateCommand(Superstructure.State.CLIMB)
//                        .alongWith(new WaitUntilCommand(() -> superstructure.hasPiece()))
//                        .incomplete

        driverXboxController.b()
                        .onTrue(new InstantCommand( () -> {
                            driveSubsystem.slowmode = !driveSubsystem.slowmode;
                            Logger.recordOutput("Drive/slowmode", driveSubsystem.slowmode);
                        }));

        driverXboxController.leftTrigger()
                .whileTrue(autoScore(4, true))
                .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
        driverXboxController.rightTrigger()
                .whileTrue(autoScore(4, false))
                .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
        driverXboxController.povUp()
                .whileTrue(autoScore(3, true))
                .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
        driverXboxController.povLeft()
                .whileTrue(autoScore(2, true))
                .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
        driverXboxController.povRight()
                .whileTrue(autoScore(3, false))
                .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
        driverXboxController.povDown()
                .whileTrue(autoScore(2, false))
                .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));


        driverXboxController.rightBumper()
                //.whileTrue(AutoScoreCommands.autoScoreProcessorCommand(driveSubsystem, superstructure))
                .onTrue(new ConditionalCommand(
                        superstructure.setWantedStateCommand(Superstructure.State.PROCESSOR_SCORE_EJECT)
                                .andThen(new WaitCommand(0.7))
                                .andThen(superstructure.setWantedStateCommand(Superstructure.State.STOW)),
                        superstructure.setWantedStateCommand(Superstructure.State.PROCESSOR_SCORE_PRE),
                        () -> superstructure.getCurrentState() == Superstructure.State.PROCESSOR_SCORE_PRE)
                );
                //.onFalse(
                        //superstructure.setWantedStateCommand(Superstructure.State.PROCESSOR_SCORE_EJECT)
                                //.andThen(new WaitCommand(0.3))
                //);
        driverXboxController.leftBumper()
                .whileTrue(AutoScoreCommands.autoScoreNetCommand(driveSubsystem, superstructure))
                //.whileTrue(superstructure.setWantedStateCommand(Superstructure.State.NET_SCORE_PRE))
                .whileFalse(new ConditionalCommand(
                        superstructure.setWantedStateCommand(Superstructure.State.STOW),
                        superstructure.setWantedStateCommand(Superstructure.State.STOW_WITH_ALGAE),
                        () -> superstructure.getCurrentState() == Superstructure.State.NET_SCORE_EJECT)
                );

        driverXboxController.y()
                .whileTrue(IntakeCommands.intakeAlgaePathClosest(driveSubsystem, superstructure));
                //.whileFalse(new ConditionalCommand(
                 //       superstructure.setWantedStateCommand(Superstructure.State.STOW),
                  //      superstructure.setWantedStateCommand(Superstructure.State.STOW_WITH_ALGAE),
                   //     () -> superstructure.getCurrentState() == Superstructure.State.STOW_WITH_ALGAE)
                //);

        driverXboxController.x()
                .onTrue(IntakeCommands.intakeFunnelCommand(superstructure));

        operatorXboxController.b()
                .whileTrue(climberSubsystem.setVoltage(() -> 8.0))
                .whileFalse(climberSubsystem.setVoltage(() -> 0.0));
        operatorXboxController.x()
                .whileTrue(climberSubsystem.setVoltage(() -> -8.0))
                .whileFalse(climberSubsystem.setVoltage(() -> 0.0));


        if(!tuning) {
            testSystemsController.leftTrigger()
                    .whileTrue(superstructure.setWantedStateCommand(Superstructure.State.L2_CORAL_PRE)
                            .andThen(superstructure.setWantedStateCommand(Superstructure.State.EJECT_CORAL)))
                    .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
            testSystemsController.rightTrigger()
                    .whileTrue(superstructure.setWantedStateCommand(Superstructure.State.L3_CORAL_PRE)
                            .andThen(superstructure.setWantedStateCommand(Superstructure.State.EJECT_CORAL)))
                    .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
            testSystemsController.leftBumper()
                    .whileTrue(superstructure.setWantedStateCommand(Superstructure.State.L4_CORAL_PRE)
                            .andThen(superstructure.setWantedStateCommand(Superstructure.State.EJECT_CORAL)))
                    .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
        }

        testSystemsController.a()
                        .whileTrue(armSubsystem.setPosition(() -> SubsystemConstants.armL4Pos))
                        .whileFalse(armSubsystem.setPosition(() -> 0.0));

        testSystemsController.b()
                        .whileTrue(endEffectorSubsystem.setVoltage(() -> 5.0))
                        .whileFalse(endEffectorSubsystem.setVoltage(() -> 0.0));

        testSystemsController.x()
                        .whileTrue(armSubsystem.setPosition(() -> SubsystemConstants.armHandoffPos))
                        .whileFalse(armSubsystem.setPosition(() -> 0.0));

//        testSystemsController.b()
//                .whileTrue(climberSubsystem.setVoltage(() -> 8.0))
//                .whileFalse(climberSubsystem.setVoltage(() -> 0.0));
//        testSystemsController.x()
//                .whileTrue(climberSubsystem.setVoltage(() -> -8.0))
//                .whileFalse(climberSubsystem.setVoltage(() -> 0.0));

        testSystemsController.rightBumper()
                .whileTrue(IntakeCommands.intakeFunnelCommand(superstructure))
        .whileFalse(superstructure.setWantedStateCommand(Superstructure.State.STOW));
        //testSystemsController.rightBumper()
         //               .whileTrue(superstructure.setWantedStateCommand())

        testSystemsController.povUp()
                .whileTrue(elevatorSubsystem.setVoltage(() -> 5.0));

        testSystemsController.povDown()
                .whileTrue(elevatorSubsystem.setVoltage(() -> -5.0));

        testSystemsController.povLeft()
                .whileTrue(armSubsystem.setVoltage(() -> -3.0));
        testSystemsController.povRight()
                .whileTrue(armSubsystem.setVoltage(() -> 3.0));

        if(tuning) {
            testSystemsController.rightTrigger()
                    .whileTrue(elevatorSubsystem.voltageRampCommand())
                    .whileFalse(elevatorSubsystem.setVoltage(() -> 0.0));
        }
    }


	public static double handleDeadBand(double value, double deadBand, boolean scale) {
		//return (Math.abs(value) > Math.abs(deadBand)) ? value : 0.0;
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

    public void autoInit() {
        superstructure.reset();
        autoCommand.schedule();
    }

    public void teleopInit() {
        autoCommand.cancel();
        superstructure.reset();
    }

    private Command autoScore(int level, boolean wantLeft) {
        return AutoScoreCommands.teleopAutoScoreCommandClosest(driveSubsystem, superstructure, () -> level, () -> wantLeft);
    }
}
