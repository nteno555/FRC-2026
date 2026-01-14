package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc2025.RobotContainer.DEAD_BAND_DRIVER;
import static frc2025.RobotContainer.handleDeadBand;
import static frc2025.subsystems.drive.DriveConstants.kSpeedAt12Volts;

public class AutoAimCommands {
    private static double modAngle(double angle) {
        angle = angle % (2*Math.PI);
        if(angle > Math.PI) return angle - 2*Math.PI;
        if(angle < -Math.PI) return angle + 2*Math.PI;
        return angle;
    }

    public static Command driveAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return robotContainer.getDrive().driveFieldRelative(
                () -> {
                    PIDController angleController = new PIDController(5, 0.0, 0.0);
                    angleController.enableContinuousInput(-Math.PI, Math.PI);

                    Pose2d robotPose = RobotState.getInstance().getVisionPose();
                    Pose2d targetPose = targetPoseSupplier.get();


                    double angleToTarget = Math.atan2(
                            targetPose.getY() - robotPose.getY(),
                            targetPose.getX() - robotPose.getX()
                    );
                    double angularVelocity = angleController.calculate(modAngle(robotPose.getRotation().getRadians()), modAngle(angleToTarget));

                    Logger.recordOutput("Drive/hubTargetPose", targetPose);
                    Logger.recordOutput("Drive/robotRotation", modAngle(robotPose.getRotation().getRadians()));
                    Logger.recordOutput("Drive/robotRotationUnmodded", robotPose.getRotation().getRadians());

                    return new ChassisSpeeds(
                            unlockedSpeeds.get().vxMetersPerSecond,
                            unlockedSpeeds.get().vyMetersPerSecond,
                            -angularVelocity
                    );
                }
        );
    }

    public static Command pivotAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier) {
        return robotContainer.getPivot().setPosition(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionPose();
            Pose2d targetPose = targetPoseSupplier.get();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().pivotTreeMap.getInterpolated(dist);
        });
    }

    public static Command shooterAutoSpinUp(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier) {
        return robotContainer.getShooter().setSpeed(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionPose();
            Pose2d targetPose = targetPoseSupplier.get();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
        });
    }

    public static Command autoAimCommand(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return shooterAutoSpinUp(robotContainer, targetPoseSupplier)
                .alongWith(driveAutoAim(robotContainer, targetPoseSupplier, unlockedSpeeds))
                .alongWith(pivotAutoAim(robotContainer, targetPoseSupplier));
    }

    // calculate the virtual pose of the hub
    public static Pose2d calculateShootingWhileMoving(RobotContainer robotContainer, ShooterTreeMap treeMap, Pose2d realTargetPose) {
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        //Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
        Pose2d virtualPose = realTargetPose;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotContainer.getDrive().getRobotRelativeSpeeds(),
            currentPose.getRotation()
        );
        Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        for (int i = 0; i < 5; i++) {
            double dx = virtualPose.getX() - currentPose.getX();
            double dy = virtualPose.getY() - currentPose.getY();
            double distance = Math.hypot(dx, dy);

            double flightTime = treeMap.tofTreeMap.getInterpolated(distance);

            virtualPose = new Pose2d(
                realTargetPose.getX() + velocity.getX() * flightTime,
                realTargetPose.getY() + velocity.getY() * flightTime,
                realTargetPose.getRotation()
            );
        }

        return virtualPose;
    }

    public static Command autoAimWhileMovingCommand(RobotContainer robotContainer, ShooterTreeMap treeMap, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return autoAimCommand(
                robotContainer,
                () -> calculateShootingWhileMoving(robotContainer, treeMap, RobotState.getInstance().getTargetPose()),
                unlockedSpeeds);
    }
}
