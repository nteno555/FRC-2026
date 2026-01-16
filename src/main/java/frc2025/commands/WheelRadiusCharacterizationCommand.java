package frc2025.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotState;
import frc2025.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterizationCommand extends Command {
    private final double gearRatio = 5.90777778;
    private final double robotRadius = Math.sqrt(2*Math.pow(10.625, 2));

    private DriveSubsystem driveSubsystem;
    private double initialGyroAngle;
    private double initialDriveMotorRotation;

    public WheelRadiusCharacterizationCommand(DriveSubsystem drive) {
        driveSubsystem = drive;
    }

    @Override
    public void initialize() {
        initialGyroAngle = RobotState.getInstance().getVisionPose().getRotation().getRadians();
        initialDriveMotorRotation = driveSubsystem.getSwerveModules()[0].getDriveMotor().getPosition().getValue().in(Units.Rotation);
    }

    @Override
    public void execute() {
        driveSubsystem.applyChassisSpeeds(
            new ChassisSpeeds(
                0.0,
                0.0,
                Math.toRadians(45)
            )
        );

        double currentGyroAngle = RobotState.getInstance().getVisionPose().getRotation().getRadians();
        double currentDriveMotorRotation = driveSubsystem.getSwerveModules()[0].getDriveMotor().getPosition().getValue().in(Units.Rotation);
        double deltaGyroAngle = currentGyroAngle - initialGyroAngle;
        double deltaDriveMotorRotation = currentDriveMotorRotation - initialDriveMotorRotation;

        double wheelRadius = (robotRadius * deltaGyroAngle) / (gearRatio * 2 * Math.PI * deltaDriveMotorRotation);
        Logger.recordOutput("Drive/Characterization/WheelRadius", wheelRadius);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.applyChassisSpeeds(
            new ChassisSpeeds(
                0.0,
                0.0,
                0.0
            )
        );
    }
}
