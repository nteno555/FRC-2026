package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.pivot.PivotSubsystem;
import frc2025.util.Alliance;

public class AutoAimCommands {
    private static double modAngle(double angle) {
        if(angle > Math.PI) return angle - 2*Math.PI;
        return angle;
    }

    public static Command driveAutoAim(RobotContainer robotContainer) {
        PIDController angleController = new PIDController(1, 0.0, 0.0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return robotContainer.getDrive().driveFieldRelative(
                () -> {
                    Pose2d robotPose = RobotState.getInstance().getVisionPose();
                    Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
                    double angleToTarget = Math.atan2(
                            targetPose.getY() - robotPose.getY(),
                            targetPose.getX() - robotPose.getX()
                    );
                    double angularVelocity = angleController.calculate(modAngle(robotPose.getRotation().getRadians()), modAngle(angleToTarget));

                    return new ChassisSpeeds(
                            0.0,
                            0.0,
                            angularVelocity
                    );
                }
        );
    }

    public static Command pivotAutoAim(RobotContainer robotContainer) {
        return robotContainer.getPivot().setPosition(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionPose();
            Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().pivotTreeMap.getInterpolated(dist);
        });
    }

    public static Command shooterAutoSpinUp(RobotContainer robotContainer) {
        return robotContainer.getShooter().setSpeed(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionPose();
            Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
        });
    }

    public static Command spinUpAndAutoAimCommand(RobotContainer robotContainer) {
        return shooterAutoSpinUp(robotContainer)
                .alongWith(driveAutoAim(robotContainer))
                .alongWith(pivotAutoAim(robotContainer));
    }
}
