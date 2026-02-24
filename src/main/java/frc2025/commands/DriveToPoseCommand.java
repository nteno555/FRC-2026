
package frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotState;
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
    private boolean ballChasing; 
    private double targetAngle;

    public DriveToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> poseSupplier, boolean ballChasing) {
        this.driveSubsystem = drive;
        this.poseSupplier = poseSupplier;
        this.ballChasing = ballChasing; 

        translationPID = new PIDController(5, 0, 0);
        rotationPID = new PIDController(3, 0, 0);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        translationDone = false;
        rotationDone = false;
        rotationPID.reset();
        translationPID.reset();   
        
        if(poseSupplier.get() == null) {
            return;
        }
        var currentPose = RobotState.getInstance().getVisionPose();
        var targetPose = poseSupplier.get();

        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        targetAngle = Math.atan2(dy,dx);
    }

    @Override
    public void execute() {
        if(poseSupplier.get() == null) {
            driveSubsystem.applyChassisSpeeds(new ChassisSpeeds());
            return;
        }
        
        var currentPose = RobotState.getInstance().getVisionPose();
        var targetPose = poseSupplier.get();

        double omegaFeedback;

        double xFeedback = translationPID.calculate(
                currentPose.getX(), targetPose.getX());
        double yFeedback = translationPID.calculate(
                currentPose.getY(), targetPose.getY());
        
        if (ballChasing){
            omegaFeedback = rotationPID.calculate(
                currentPose.getRotation().getRadians(), targetAngle
            );
        }
        else {
            omegaFeedback = rotationPID.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        }

        xFeedback = MathUtil.clamp(Math.pow(Math.abs(xFeedback), 1), 0, 3) * Math.signum(xFeedback);
        yFeedback = MathUtil.clamp(Math.pow(Math.abs(yFeedback), 1), 0, 3) * Math.signum(yFeedback);

        double dist = Math.sqrt(Math.pow(currentPose.getX() - targetPose.getX(), 2) + Math.pow(currentPose.getY() - targetPose.getY(), 2));
        Logger.recordOutput("Drive/PathFollowingError", dist);

        if(dist < 0.05)
            translationDone = true;
        
        if(ballChasing) {
            if(Math.abs(currentPose.getRotation().getRadians() - targetAngle) < 0.1) 
                rotationDone = true;
        }
        else if(Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians()) < 0.1)
            rotationDone = true;

        if(translationDone) {
            xFeedback = 0;
            yFeedback = 0;
        }
        if(rotationDone) {
            omegaFeedback = 0;
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xFeedback,
                yFeedback,
                omegaFeedback,
                currentPose.getRotation()
        );

        driveSubsystem.applyChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.applyChassisSpeeds(new ChassisSpeeds()); // Stop motors
    }

    @Override
    public boolean isFinished() {
        return translationDone && rotationDone;
    }
}
