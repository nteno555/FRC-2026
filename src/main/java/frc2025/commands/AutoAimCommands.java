package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

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

    public static Command driveAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return robotContainer.getDrive().driveFieldRelative(
                () -> {
                    PIDController angleController = new PIDController(5, 0.0, 0.0);
                    angleController.enableContinuousInput(-Math.PI, Math.PI);

                    Pose2d shooterPose = RobotState.getInstance().getVisionShooterPose();
                    Pose2d targetPose = targetPoseSupplier.get();

                    double angleToTarget = RobotState.getInstance().getAngleToVirtualTarget();
                    double angularVelocity = angleController.calculate(modAngle(shooterPose.getRotation().getRadians()), modAngle(angleToTarget));

                    Logger.recordOutput("Drive/hubTargetPose", targetPose);
                    Logger.recordOutput("Drive/robotRotation", modAngle(shooterPose.getRotation().getRadians()));
                    Logger.recordOutput("Drive/robotRotationUnmodded", shooterPose.getRotation().getRadians());

                    return new ChassisSpeeds(
                            unlockedSpeeds.get().vxMetersPerSecond,
                            unlockedSpeeds.get().vyMetersPerSecond,
                            -angularVelocity
                    );
                }
        );
    }

   public static Command autoIndex(RobotContainer robotContainer) {
        BooleanSupplier readySupplier = () -> {
            boolean good = true;
//            if(!SubsystemConstants.hasTurret) {
//                good = good && RobotState.getInstance().getShooterReady() && RobotState.getInstance().getDriveShotReady();
//            } else {
//                good = good && RobotState.getInstance().getShooterReady() && robotContainer.getTurret().getShotReady();
//            }

//            if(DriverStation.isAutonomous()) good = good && RobotState.getInstance().getDriveAutoAiming();
//            good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.autoAimMaxSpeed);

            Pose2d pose = RobotState.getInstance().getVisionPose();
            Pose2d virtualPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            boolean inBlueZone = pose.getX() < 3.0;
            boolean inRedZone = pose.getX() > 13.25;

            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                good = good && inBlueZone;
            } else {
                good = good && inRedZone;
            }

            good = good && (pose.getTranslation().getDistance(virtualPose.getTranslation()) < ShooterTreeMap.getInstance().getMaxDistance());

            return good;
        };

        return robotContainer.getIndexer().setVoltage(() -> {
            Logger.recordOutput("Indexer/autoIndexShooterReady", RobotState.getInstance().getShooterReady());
            Logger.recordOutput("Indexer/autoIndexDriveReady", RobotState.getInstance().getDriveShotReady());

            if(readySupplier.getAsBoolean()) {
                Logger.recordOutput("Indexer/ready", true);
                return SubsystemConstants.spindexerShootVoltage;
            } else {
                Logger.recordOutput("Indexer/ready", false);
                return 0.0;
            }
        }, () -> {
            if (readySupplier.getAsBoolean()) {
                return SubsystemConstants.towerShootVoltage;
            }
            else {
                return 0.0;
            }
        });
   }
}
