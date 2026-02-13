package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
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
            boolean good;
            if(!SubsystemConstants.hasTurret) {
                good = robotContainer.getShooter().getShotReady() && robotContainer.getDrive().getShotReady();
            } else {
                good = robotContainer.getShooter().getShotReady() && robotContainer.getTurret().getShotReady();
            }

            Pose2d pose = RobotState.getInstance().getVisionShooterPose();
            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = pose.getTranslation().getDistance(targetPose.getTranslation());
            good = good && (dist < ShooterTreeMap.getInstance().getMaxDistance());

            if(DriverStation.isAutonomous()) {
                boolean inBlueZone = pose.getX() < FieldConstants.blueTrenchX;
                boolean inRedZone = pose.getX() > FieldConstants.redTrenchX;

                if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                    Logger.recordOutput("Indexer/autoIndex/autoInZone", inBlueZone);
                    good = good && inBlueZone;
                } else {
                    Logger.recordOutput("Indexer/autoIndex/autoInZone", inRedZone);
                    good = good && inRedZone;
                }
            }

            double a = RobotState.getInstance().getPigeonAcceleration();
            good = good && (a < SubsystemConstants.autoAimMaxAccel);
            good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.autoAimMaxSpeed);

            Logger.recordOutput("Indexer/autoIndex/accelReady", (a < SubsystemConstants.autoAimMaxAccel));
            Logger.recordOutput("Indexer/autoIndex/shooterReady", robotContainer.getShooter().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/turretReady", robotContainer.getTurret().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/speedReady", (RobotState.getInstance().getSpeed() < SubsystemConstants.autoAimMaxSpeed));
            Logger.recordOutput("Indexer/autoIndex/speed", RobotState.getInstance().getSpeed());
            Logger.recordOutput("Indexer/autoIndex/accel", a);
            Logger.recordOutput("Indexer/autoIndexReady", good);

            return good;
        };

        return robotContainer.getIndexer().setVoltage(() -> {
            if(readySupplier.getAsBoolean()) {
                return SubsystemConstants.spindexerShootVoltage;
            } else {
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
