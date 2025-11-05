package frc2025.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.Robot;
import frc2025.RobotState;
import frc2025.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc2025.subsystems.drive.DriveSubsystem.getPPConfig;

public class FollowPathCommand extends Command {
    private final Timer timer = new Timer();
    private final DriveSubsystem driveSubsystem;
    private final Supplier<PathPlannerPath> pathSupplier;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;

    private PIDController rotationPID;

    private PIDController translationPID;
    private boolean useDynamicPath;
    private Supplier<Boolean> flip;
    private boolean useTimeout = false;
    private boolean translationDone = false;
    private boolean doAllianceFlip = false;
    private double distanceThresh = 0.15;

    public FollowPathCommand(DriveSubsystem drive, Supplier<PathPlannerPath> pathSupplier, Supplier<Boolean> flip) {
        this.pathSupplier = pathSupplier;
        this.driveSubsystem = drive;
        this.useDynamicPath = true;
        this.flip = flip;

        translationPID = new PIDController(1.2, 0, 0);
        rotationPID = new PIDController(1.2, 0, 0);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.driveSubsystem);
        useTimeout = false;
    }

    public FollowPathCommand(DriveSubsystem drive, Supplier<PathPlannerPath> pathSupplier, Supplier<Boolean> flip, boolean useTimeout) {
        this.pathSupplier = pathSupplier;
        this.driveSubsystem = drive;
        this.useDynamicPath = true;
        this.flip = flip;

        if(!useTimeout) {
            translationPID = new PIDController(1.2, 0, 0);
            rotationPID = new PIDController(3, 0, 0);
            rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        } else {
            translationPID = new PIDController(3, 0, 0);
            rotationPID = new PIDController(3, 0, 0);
            rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        }

        addRequirements(this.driveSubsystem);
        this.useTimeout = useTimeout;
    }

    public FollowPathCommand(DriveSubsystem drive, Supplier<PathPlannerPath> pathSupplier, Supplier<Boolean> flip, double thresh) {
        this.pathSupplier = pathSupplier;
        this.driveSubsystem = drive;
        this.useDynamicPath = true;
        this.flip = flip;

        translationPID = new PIDController(1.2, 0, 0);
        rotationPID = new PIDController(3, 0, 0);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.driveSubsystem);
        this.useTimeout = false;
        this.distanceThresh = thresh;
    }

    /*public FollowPathCommand withAllianceFlipping() {
        this.doAllianceFlip = true;
        return this;
    }*/

    @Override
    public void initialize() {
        rotationPID.reset();
        translationPID.reset();

        path = pathSupplier.get();
        if(this.flip.get())
            path = path.flipPath();

        trajectory = path.generateTrajectory(
                driveSubsystem.getRobotRelativeSpeeds(), RobotState.getInstance().getVisionPose().getRotation(), getPPConfig());
        translationDone = false;
        this.timer.restart();

        Logger.recordOutput("Drive/followPathEnd", new Pose2d(path.getPathPoses().get(path.getPathPoses().size()-1).getTranslation(), path.getGoalEndState().rotation()));
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerTrajectoryState desiredState = trajectory.sample(currentTime);
        var currentPose = RobotState.getInstance().getVisionPose();

        //if(this.timer.get() > this.trajectory.getTotalTimeSeconds())
        //    translationPID = new PIDController(3, 0, 0);
        //else translationPID = new PIDController(0.5, 0, 0.1);
        /*translationPID.setP(
                1.5 +
                MathUtil.clamp((this.trajectory.getTotalTimeSeconds() - this.timer.get()) / this.trajectory.getTotalTimeSeconds(), 0, 1) * 1.5
        );*/

        double xFeedback = translationPID.calculate(
                currentPose.getX(), desiredState.pose.getX());
        double yFeedback = translationPID.calculate(
                currentPose.getY(), desiredState.pose.getY());
        double omegaFeedback = rotationPID.calculate(
                currentPose.getRotation().getRadians(), desiredState.pose.getRotation().getRadians());

        double dist = Math.sqrt(Math.pow(currentPose.getX() - desiredState.pose.getX(), 2) + Math.pow(currentPose.getY() - desiredState.pose.getY(), 2));

        RobotState.getInstance().setAlignDist(dist);

        // REPLANNING MAY CAUSE BAD BEHAVIOR IDK
        /*if(dist > 0.2) {
            path = path.
            trajectory = path.generateTrajectory(
                    driveSubsystem.getRobotRelativeSpeeds(), RobotState.getInstance().getVisionPose().getRotation(), getPPConfig());
            this.timer.reset();
        }*/

        Logger.recordOutput("Drive/PathFollowingError", dist);

        if(this.timer.get() > this.trajectory.getTotalTimeSeconds()
                && dist < distanceThresh)
            translationDone = true;

        if(translationDone) {
            xFeedback = 0;
            yFeedback = 0;
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredState.fieldSpeeds.plus(new ChassisSpeeds(xFeedback, yFeedback, omegaFeedback)),
                currentPose.getRotation()
        );

        driveSubsystem.applyChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop(); // Stop timer
        driveSubsystem.applyChassisSpeeds(new ChassisSpeeds()); // Stop motors
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        Pose2d endPose = trajectory.getEndState().pose;

        if(!useTimeout) {
            // check reef side to side distance
            Translation2d translation = currentPose.getTranslation().minus(endPose.getTranslation());
            Translation2d strafe = translation.plus(RobotState.getInstance().getReefDepthVector());
            double rotationDistance = Math.abs(currentPose.getRotation().minus(endPose.getRotation()).getDegrees());
            //rotationDistance = 0;
            Logger.recordOutput("Vision/alignDistance", translation.getNorm());
            Logger.recordOutput("Drive/targetPose", endPose);

            //return (translation.getNorm() < 0.05 && strafe.getNorm() < 0.02 && rotationDistance < 2);
            double velocity = Math.pow(driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond, 2) + Math.pow(driveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond, 2);
            velocity = Math.sqrt(velocity);
            double angularVelocity = driveSubsystem.getRobotRelativeSpeeds().omegaRadiansPerSecond;
            return (translationDone && rotationDistance < 3) && this.timer.hasElapsed(trajectory.getTotalTimeSeconds());// && velocity < 0.02 && angularVelocity < 0.1;
        }
        return this.timer.hasElapsed(trajectory.getTotalTimeSeconds() + 0.2);
    }
}
