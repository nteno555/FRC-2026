package frc2025;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveIOReal;
import frc2025.subsystems.drive.DriveIOSim;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.vision.VisionIOReal;
import frc2025.subsystems.vision.VisionIOSim;
import frc2025.subsystems.vision.VisionSubsystem;
import frc2025.util.AlignPathBuilder;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import static frc2025.subsystems.drive.DriveConstants.*;

public class RobotContainer {
    public static final double DEAD_BAND_DRIVER = 0.1;
    public static final double DEAD_BAND_OPERATOR = 0.1;
    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);
    private final CommandXboxController testSystemsController = new CommandXboxController(2);

    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    Autos autos;
    Command autoCommand;
    private boolean tuning = false;

    public RobotContainer() {
        createSubsystems();
        configureBindings();
        AlignPathBuilder.initialize();
        autos = new Autos(driveSubsystem);
        autoCommand = autos.Taxi();
    }

    public void periodic() {
        int y = 0;
    }

    private void createSubsystems() {
        // github slack bot please work
        if(RobotBase.isReal()) {
            driveSubsystem = new DriveSubsystem(new DriveIOReal(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
            visionSubsystem = new VisionSubsystem(new VisionIOReal());
        } else {
             driveSubsystem = new DriveSubsystem(new DriveIOSim(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight));
             visionSubsystem = new VisionSubsystem(new VisionIOSim());
         }

    }
    private void configureBindings() {
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

    public void autoInit() {
        autoCommand.schedule();
    }

    public void teleopInit() {
        autoCommand.cancel();
    }
}
