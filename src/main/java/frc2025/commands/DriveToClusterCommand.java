package frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotState;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.vision.VisionSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class DriveToClusterCommand extends Command {

    private final DriveSubsystem driveSubsystem;

    private PIDController yawPID;
    private boolean reachedBall;
    private final double clusterThreshold = 8.0;
    private final double areaThreshold = 3.0;

    public DriveToClusterCommand(DriveSubsystem drive) {
        this.driveSubsystem = drive;

        yawPID = new PIDController(0.3, 0, 0);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        yawPID.reset();
        reachedBall = false;
    }

    @Override
    public void execute() {

        List<PhotonTrackedTarget> balls = RobotState.getInstance().getBallTargets();

        if (balls.isEmpty()) {
            driveSubsystem.applyChassisSpeeds(new ChassisSpeeds(0,0,1.0));
            return;
        }

        List<List<PhotonTrackedTarget>> clusters = new ArrayList<>();


        for (PhotonTrackedTarget ball : balls) {

            boolean added = false;

            for (List<PhotonTrackedTarget> cluster : clusters) {

                double clusterYaw = cluster.get(0).getYaw();

                if (Math.abs(ball.getYaw() - clusterYaw) < clusterThreshold) {
                    cluster.add(ball);
                    added = true;
                    break;
                }
            }

            if (!added) {
                List<PhotonTrackedTarget> newCluster = new ArrayList<>();
                newCluster.add(ball);
                clusters.add(newCluster);
            }
        }

        List<PhotonTrackedTarget> bestCluster = clusters.get(0);

        for (List<PhotonTrackedTarget> cluster : clusters) {
            if (cluster.size() > bestCluster.size()) {
                bestCluster = cluster;
            }
        }

        PhotonTrackedTarget targetBall = bestCluster.get(0);

        for (PhotonTrackedTarget ball : bestCluster) {
            if (ball.getArea() > targetBall.getArea()) {
                targetBall = ball;
            }
        }

        double yaw = targetBall.getYaw();
        double area = targetBall.getArea();

        double omega = yawPID.calculate(yaw, 0);
        omega = MathUtil.clamp(omega, -2, 2);

        double speed = 2.0;

        if (area > areaThreshold) {
            reachedBall = true;
            speed = 0;
        }

        double yawRad = Math.toRadians(yaw);

        double vx = speed * Math.cos(yawRad);
        double vy = speed * Math.sin(yawRad);

        ChassisSpeeds speeds = new ChassisSpeeds(
                vx,
                vy,
                omega
        );

        driveSubsystem.applyChassisSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.applyChassisSpeeds(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return reachedBall;
    }
}