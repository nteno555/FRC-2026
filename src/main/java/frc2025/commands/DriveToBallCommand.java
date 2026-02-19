
package frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotState;
import frc2025.subsystems.drive.DriveSubsystem;

public class DriveToBallCommand extends Command {

    private final DriveSubsystem drive;

    // tune
    private final PIDController yawPID; 
    private static final double maxForward = 2.0;
    private static final double maxOmega = 3.0;

    private static final double speedScaleFactor = 10.0;

    private static final double alignTolerance = 4.0;
    private static final double areaThreshold = 13.0;

    public DriveToBallCommand(DriveSubsystem drive) {
        this.drive = drive;

        yawPID = new PIDController(0.03, 0, 0);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        yawPID.reset();
    }

    @Override
    public void execute() {

        var target = RobotState.getInstance().getBallTarget();

        if (target == null) {
            drive.applyChassisSpeeds(new ChassisSpeeds());
            return;
        }

        double yaw = target.getYaw();
        double area = target.getArea();

        double omega = yawPID.calculate(yaw, 0);
        omega = MathUtil.clamp(omega, -maxOmega, maxOmega);

        double forward = 0.0;

        if (Math.abs(yaw) < alignTolerance) {
            forward = (1.0 / (area)) * speedScaleFactor;
            forward = MathUtil.clamp(forward, 0, maxForward);
        }

        ChassisSpeeds speeds = new ChassisSpeeds(forward ,0, omega);

        drive.applyChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {

        var target = RobotState.getInstance().getBallTarget();

        if (target == null) {
            return false;
        }

        return (target.getArea() > areaThreshold);
    }

    @Override
    public void end(boolean interrupted) {
        drive.applyChassisSpeeds(new ChassisSpeeds());
    }
}

