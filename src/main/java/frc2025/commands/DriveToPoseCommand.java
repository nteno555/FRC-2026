
package frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private PIDController rotationPID;
    private PIDController translationPID;
    private Supplier<Pose2d> poseSupplier;
    private boolean translationDone;
    private boolean rotationDone;

    public DriveToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> poseSupplier) {
        this.driveSubsystem = drive;
        this.poseSupplier = poseSupplier;

        translationPID = new PIDController(SubsystemConstants.translationkP, SubsystemConstants.translationkI,
                SubsystemConstants.translationkD);
        rotationPID = new PIDController(SubsystemConstants.rotationkP, SubsystemConstants.rotationkI,
                SubsystemConstants.rotationkD);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        translationDone = false;
        rotationDone = false;
        rotationPID.reset();
        translationPID.reset();
    }

    @Override
    public void execute() {
        var currentPose = RobotState.getInstance().getVisionPose();
        var desiredPose = poseSupplier.get();

        double xFeedback = translationPID.calculate(
                currentPose.getX(), desiredPose.getX());
        double yFeedback = translationPID.calculate(
                currentPose.getY(), desiredPose.getY());
        double omegaFeedback = rotationPID.calculate(
                currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

        xFeedback = MathUtil.clamp(Math.pow(Math.abs(xFeedback), 1), 0, 3) * Math.signum(xFeedback);
        yFeedback = MathUtil.clamp(Math.pow(Math.abs(yFeedback), 1), 0, 3) * Math.signum(yFeedback);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xFeedback,
                yFeedback,
                omegaFeedback,
                currentPose.getRotation());

        driveSubsystem.applyChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.applyChassisSpeeds(new ChassisSpeeds()); // Stop motors
    }

    @Override
    public boolean isFinished() {
        var currentPose = RobotState.getInstance().getVisionPose();
        var desiredPose = poseSupplier.get();

        double distance = Math.sqrt(Math.pow(currentPose.getX() - desiredPose.getX(), 2)
                + Math.pow(currentPose.getY() - desiredPose.getY(), 2));
        double angle = Math.abs(currentPose.getRotation().getRadians() - desiredPose.getRotation().getRadians());

        return distance < 0.15 && angle < Math.toRadians(15);
    }
}
