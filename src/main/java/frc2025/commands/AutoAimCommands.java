package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoAimCommands {
    private static double modAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle > Math.PI)
            return angle - 2 * Math.PI;
        if (angle < -Math.PI)
            return angle + 2 * Math.PI;
        return angle;
    }

    public static Command driveAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier,
            Supplier<ChassisSpeeds> unlockedSpeeds) {
        return robotContainer.getDrive().driveFieldRelative(
                () -> {
                    PIDController angleController = new PIDController(3, 0.0, 0.0);
                    angleController.enableContinuousInput(-Math.PI, Math.PI);

                    Pose2d shooterPose = RobotState.getInstance().getVisionShooterPose();
                    Pose2d targetPose = targetPoseSupplier.get();

                    double angleToTarget = RobotState.getInstance().getAngleToVirtualTarget();
                    double angularVelocity = angleController.calculate(modAngle(shooterPose.getRotation().getRadians()),
                            modAngle(angleToTarget));

//                    Logger.recordOutput("Drive/hubTargetPose", targetPose);
//                    Logger.recordOutput("Drive/robotRotation", modAngle(shooterPose.getRotation().getRadians()));
//                    Logger.recordOutput("Drive/robotRotationUnmodded", shooterPose.getRotation().getRadians());

                    return new ChassisSpeeds(
                            unlockedSpeeds.get().vxMetersPerSecond,
                            unlockedSpeeds.get().vyMetersPerSecond,
                            angularVelocity);
                });
    }

    // For Auto
    public static Command autoIndex(RobotContainer robotContainer) {
        return Commands.none();
    }

    public static Command autoIndexTeleop(RobotContainer robotContainer, Supplier<Boolean> hasTurret) {
        return Commands.none();
    }
}
