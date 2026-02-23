
package frc2025.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotState;
import frc2025.subsystems.drive.DriveSubsystem;

public class DriveToClusterCommand extends Command {

    private final DriveSubsystem drive;

    // tune
    private final PIDController yawPID; 
    private static final double maxForward = 2.0;
    private static final double maxOmega = 3.0;

    private static final double speedScaleFactor = 10.0;

    private static final double alignTolerance = 4.0;
    private static final double areaThreshold = 13.0;

    private static final double searchOmega = 0.8;

    public DriveToClusterCommand(DriveSubsystem drive) {
        this.drive = drive;

        yawPID = new PIDController(0.2, 0, 0);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        yawPID.reset();
    }

    public Pair<Double, Double> calculateClusterYawAndTotalArea(List<PhotonTrackedTarget> targets) {
        double sumX = 0.0;
        double sumY = 0.0;
        double totalArea = 0.0;

        for (PhotonTrackedTarget t : targets) {
            double yawRad = Math.toRadians(t.getYaw());
            double area = t.getArea();

            sumX += area * Math.cos(yawRad);
            sumY += area * Math.sin(yawRad);
            totalArea += area;
        }

        if (totalArea == 0) {
            return new Pair<>(0.0, 0.0);
        }

        double clusterYawRad = Math.atan2(sumY, sumX);
        double clusterYawDeg = Math.toDegrees(clusterYawRad);

        return new Pair<>(clusterYawDeg, totalArea);
    }

    @Override
    public void execute() {

        List<PhotonTrackedTarget> targets = RobotState.getInstance().getBallTargets();

        if (targets.isEmpty()) {
            drive.applyChassisSpeeds(new ChassisSpeeds(0, 0, searchOmega));
            return;
        }
        
        Pair<Double, Double> stats = calculateClusterYawAndTotalArea(targets);
        double yaw = stats.getFirst();
        double area = stats.getSecond();

        if (area == 0) {
            drive.applyChassisSpeeds(new ChassisSpeeds());
            return;
        }

        double omega = yawPID.calculate(yaw, 0);
        omega = MathUtil.clamp(omega, -maxOmega, maxOmega);

        Logger.recordOutput("Cluster/clusterYaw", yaw);
        Logger.recordOutput("Cluster/clusterYaw", area);
        Logger.recordOutput("Cluster/omega", omega);

        double speed = 0.0;
        double vx = 0.0;
        double vy = 0.0;

        if (Math.abs(yaw) < alignTolerance) {
            speed = speedScaleFactor / (area + 1.0);
            speed = MathUtil.clamp(speed, 0.2, maxForward);
        }

        double yawRad = Math.toRadians(yaw);
        vx = speed * Math.cos(yawRad);
        vy = speed * Math.sin(yawRad);

        Logger.recordOutput("Cluster/forward", vx);
        Logger.recordOutput("Cluster/side", vy);

        ChassisSpeeds speeds = new ChassisSpeeds(-vx ,-vy, omega);

        drive.applyChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {

        List<PhotonTrackedTarget> targets = RobotState.getInstance().getBallTargets();

        if (targets.isEmpty()) {
            return false;
        }

        double area = calculateClusterYawAndTotalArea(targets).getSecond();

        return (area > areaThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        drive.applyChassisSpeeds(new ChassisSpeeds());
    }
}

