package frc2025.commands;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2025.Robot;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Supplier;

import static frc2025.subsystems.drive.DriveSubsystem.getPPConfig;

public class FollowPathCommand extends Command {
    private final Timer timer = new Timer();
    private final DriveSubsystem driveSubsystem;
    private final Supplier<PathPlannerPath> pathSupplier;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private List<EventMarker> eventMarkers;
    private boolean autoAimOverride;

    private double distanceTraveled;
    private Pose2d lastPose;
    private List<String> firedMarkers;

    private PIDController rotationPID;

    private PIDController translationPID;
    private Supplier<Boolean> flip;
    private boolean useTimeout = false;

    public FollowPathCommand(DriveSubsystem drive, Supplier<PathPlannerPath> pathSupplier, Supplier<Boolean> flip, boolean useTimeout) {
        this.pathSupplier = pathSupplier;
        this.driveSubsystem = drive;
        this.flip = flip;

        translationPID = new PIDController(2.5, 0, 0);
        rotationPID = new PIDController(3.0, 0, 0);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.driveSubsystem);
        this.useTimeout = useTimeout;
        this.autoAimOverride = true;
        this.firedMarkers = new ArrayList<>();
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        translationPID.reset();

        path = pathSupplier.get();
        if(this.flip.get()) {
            path = path.flipPath();
        }

        this.eventMarkers = path.getEventMarkers();

        trajectory = path.generateTrajectory(
                driveSubsystem.getRobotRelativeSpeeds(), RobotState.getInstance().getVisionPose().getRotation(), getPPConfig());
        this.timer.restart();

        this.distanceTraveled = 0;
        this.lastPose = trajectory.sample(0).pose;
        this.firedMarkers.clear();

        Logger.recordOutput("Drive/followPathEnd", new Pose2d(path.getPathPoses().get(path.getPathPoses().size()-1).getTranslation(), path.getGoalEndState().rotation()));
    }

    private double modAngle(double angle) {
        angle = angle % (2*Math.PI);
        if(angle > Math.PI) return angle - 2*Math.PI;
        if(angle < -Math.PI) return angle + 2*Math.PI;
        return angle;
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerTrajectoryState desiredState = trajectory.sample(currentTime);
        var currentPose = RobotState.getInstance().getVisionPose();

        double xFeedback = translationPID.calculate(
                currentPose.getX(), desiredState.pose.getX());
        double yFeedback = translationPID.calculate(
                currentPose.getY(), desiredState.pose.getY());
        double omegaFeedback = rotationPID.calculate(
                currentPose.getRotation().getRadians(), desiredState.pose.getRotation().getRadians());

        double pathProgress = (currentTime/trajectory.getTotalTimeSeconds()) * (path.getWaypoints().size() - 1);
        pathProgress = Math.min(pathProgress, path.getWaypoints().size()-1);
        Logger.recordOutput("Drive/pathProgress", pathProgress);

        for(EventMarker marker : eventMarkers) {
            Logger.recordOutput("Drive/markerStart", marker.position());
            Logger.recordOutput("Drive/markerEnd", marker.endPosition());
            if (pathProgress >= marker.position() && pathProgress <= marker.endPosition()) {
                autoAimOverride = !marker.triggerName().equals("stop auto aim");
            } else {
                autoAimOverride = true;
            }
            Logger.recordOutput("Drive/autoAutoAimOverride", autoAimOverride);
        }

        autoAimOverride = (SubsystemConstants.hasTurret);

        Logger.recordOutput("Drive/pathProgress", pathProgress);
        Logger.recordOutput("Drive/FollowPath/wantedVelocity", desiredState.linearVelocity);
        Logger.recordOutput("Drive/FollowPath/currentVelocity", RobotState.getInstance().getSpeed());

        if(!SubsystemConstants.hasTurret && autoAimOverride) {
            double angleToTarget = RobotState.getInstance().getAngleToVirtualTarget();
            omegaFeedback = rotationPID.calculate(modAngle(currentPose.getRotation().getRadians()), modAngle(angleToTarget));
        }

        double dist = Math.sqrt(Math.pow(currentPose.getX() - desiredState.pose.getX(), 2) + Math.pow(currentPose.getY() - desiredState.pose.getY(), 2));

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
                desiredState.fieldSpeeds.vxMetersPerSecond,
                desiredState.fieldSpeeds.vyMetersPerSecond,
                desiredState.fieldSpeeds.omegaRadiansPerSecond
//                omegaFeedback * -1
        );

        if(!SubsystemConstants.hasTurret && autoAimOverride) {
            desiredSpeeds.omegaRadiansPerSecond = 0.0;
        }

        Logger.recordOutput("Drive/PathFollowingError", dist);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredSpeeds.plus(new ChassisSpeeds(xFeedback, yFeedback, omegaFeedback)),
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
            Translation2d translation = currentPose.getTranslation().minus(endPose.getTranslation());
            double rotationDistance = Math.abs(currentPose.getRotation().minus(endPose.getRotation()).getDegrees());
            Logger.recordOutput("Vision/alignDistance", translation.getNorm());
            Logger.recordOutput("Drive/targetPose", endPose);

            return (translation.getNorm() < 0.2 && rotationDistance < 10) && RobotState.getInstance().getSpeed() < 0.5;// && velocity < 0.02 && angularVelocity < 0.1;
        }
        return this.timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}