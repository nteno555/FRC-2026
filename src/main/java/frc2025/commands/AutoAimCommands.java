package frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.Robot;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;
import frc2025.RobotContainer;
import java.sql.Driver;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoAimCommands {
    private static double modAngle(double angle) {
        angle = angle % (2*Math.PI);
        if(angle > Math.PI) return angle - 2*Math.PI;
        if(angle < -Math.PI) return angle + 2*Math.PI;
        return angle;
    }

    public static Command driveAutoAim(RobotContainer robotContainer) {
        Supplier<Double> omegaSpeed = () -> {
            Pose2d virtualTarget = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            if (virtualTarget == null) {
                return 0.0;
            }
            
            Pose2d shooterPose = RobotState.getInstance().getDelayedShooterPose();

            double desiredHeading = Math.atan2(
                    virtualTarget.getY() - shooterPose.getY(),
                    virtualTarget.getX() - shooterPose.getX()
            );

            double currentHeading = RobotState.getInstance().getVisionPose().getRotation().getRadians();
            double angleError = modAngle(desiredHeading - currentHeading);

            double feedback = SubsystemConstants.alignRotationkP * angleError;
            double feedforward = RobotState.getInstance().getVirtualTargetAngularVelocity();

            double omega = feedforward + feedback;
            return MathUtil.clamp(omega, -SubsystemConstants.autoAimMaxAngularVelocity, SubsystemConstants.autoAimMaxAngularVelocity);
        };

        return robotContainer.getDrive().applyChassisSpeedsCommand(() -> new ChassisSpeeds(0, 0, omegaSpeed.get()));
    }


}