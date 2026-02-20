
package frc2025.commands;

import java.util.List;

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

        yawPID = new PIDController(0.03, 0, 0);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        yawPID.reset();
    }

    public Pair<Double, Double> calculateAverageYawAndTotalArea(List<PhotonTrackedTarget> targets) {
        double weightedYawSum = 0.0;
        double totalArea = 0.0;
        for (int i = 0; i < targets.size(); i++) {
            double yaw = targets.get(i).getYaw();
            double area = targets.get(i).getArea();
            weightedYawSum += (yaw * area);
            totalArea += area;
        }

        if (totalArea == 0) {
            return new Pair<>(0.0, 0.0);
        }

        return new Pair<>((weightedYawSum) / (totalArea), totalArea);
    }

    @Override
    public void execute() {

        List<PhotonTrackedTarget> targets = RobotState.getInstance().getBallTargets();

        if (targets.isEmpty()) {
            drive.applyChassisSpeeds(new ChassisSpeeds(0, 0, searchOmega));
            return;
        }
        
        Pair<Double, Double> stats = calculateAverageYawAndTotalArea(targets);
        double yaw = stats.getFirst();
        double area = stats.getSecond();

        if (area == 0) {
            drive.applyChassisSpeeds(new ChassisSpeeds());
            return;
        }

        double omega = yawPID.calculate(yaw, 0);
        omega = MathUtil.clamp(omega, -maxOmega, maxOmega);

        double forward = 0.0;


        if (Math.abs(yaw) < alignTolerance) {
            forward = speedScaleFactor / (area + 1.0);
            forward = MathUtil.clamp(forward, 0.2, maxForward);
        }

        ChassisSpeeds speeds = new ChassisSpeeds(forward ,0, omega);

        drive.applyChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {

        List<PhotonTrackedTarget> targets = RobotState.getInstance().getBallTargets();

        if (targets.isEmpty()) {
            return false;
        }

        double area = calculateAverageYawAndTotalArea(targets).getSecond();

        return (area > areaThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        drive.applyChassisSpeeds(new ChassisSpeeds());
    }
}

