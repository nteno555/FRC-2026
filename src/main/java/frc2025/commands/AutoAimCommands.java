package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
        angle = angle % (2*Math.PI);
        if(angle > Math.PI) return angle - 2*Math.PI;
        if(angle < -Math.PI) return angle + 2*Math.PI;
        return angle;
    }

    public static Command turretAutoAim(RobotContainer robotContainer) {
        return robotContainer.getTurret().autoAim()
                .alongWith(robotContainer.getHood().autoAim())
                .alongWith(robotContainer.getShooter().autoAim());
    }

    public static Command driveAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return robotContainer.getDrive().driveFieldRelative(
                () -> {
                    PIDController angleController = new PIDController(3, 0.0, 0.0);
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
                            angularVelocity
                    );
                }
        );
    }

    // For Auto
   public static Command autoIndex(RobotContainer robotContainer) {
        BooleanSupplier readySupplier = () -> {
            boolean good;
            if(!SubsystemConstants.hasTurret) {
                good = robotContainer.getShooter().getShotReady() && robotContainer.getDrive().getShotReady() && robotContainer.getHood().getShotReady();
            } else {
                good = robotContainer.getShooter().getShotReady() && robotContainer.getTurret().getShotReady() && robotContainer.getHood().getShotReady();
            }

            Pose2d pose = RobotState.getInstance().getVisionShooterPose();

            boolean inBlueZone = pose.getX() < FieldConstants.inBlueZoneX && !Alliance.get();
            boolean inRedZone = pose.getX() > FieldConstants.inRedZoneX && Alliance.get();
            good = good && (inBlueZone || inRedZone);

            Logger.recordOutput("Indexer/autoIndex/shooterReady", robotContainer.getShooter().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/turretReady", robotContainer.getTurret().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/speed", RobotState.getInstance().getSpeed());
            Logger.recordOutput("Indexer/autoIndexReady", good);

            return good;
        };

        return robotContainer.getIndexer().setVoltage(() -> {
            if(readySupplier.getAsBoolean()) {
                return SubsystemConstants.indexerShootVoltage;
            } else {
                return 0.0;
            }
        });
   }
    public static Command autoIndex(RobotContainer robotContainer, Supplier<Boolean> hasTurret) {
        BooleanSupplier readySupplier = () -> {
            boolean good;

            if(!hasTurret.get()) {
                good = robotContainer.getShooter().getShotReady() && robotContainer.getDrive().getShotReady() && robotContainer.getHood().getShotReady();
            } else {
                good = robotContainer.getShooter().getShotReady() && robotContainer.getTurret().getShotReady() && robotContainer.getHood().getShotReady();
            }

            if (RobotState.getInstance().getShuttling() && !DriverStation.isAutonomousEnabled()) {
                return good;
            }

            Pose2d pose = RobotState.getInstance().getVisionShooterPose();
            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = pose.getTranslation().getDistance(targetPose.getTranslation());
            double xPos = pose.getX();
            double yPos = pose.getY();

            boolean nearBlueTrench = Math.abs(xPos - FieldConstants.blueTrenchX) < FieldConstants.trenchThreshold;
            boolean nearRedTrench = Math.abs(xPos - FieldConstants.redTrenchX) < FieldConstants.trenchThreshold;
            boolean onBumpTrench = (yPos > FieldConstants.highBumpY) || (yPos < FieldConstants.lowBumpY);

            good = good && !((nearBlueTrench || nearRedTrench) && onBumpTrench);

            boolean inZone;
            boolean inClimb;

            if (Alliance.get()) {
                inZone = xPos > FieldConstants.inRedZoneX;
                inClimb = (xPos > FieldConstants.redClimbX) && (yPos > FieldConstants.redClimbLowY) && (yPos < FieldConstants.redClimbHighY);
            }
            else {
                inZone = xPos < FieldConstants.inBlueZoneX;
                inClimb = (xPos < FieldConstants.blueClimbX) && (yPos > FieldConstants.blueClimbLowY) && (yPos < FieldConstants.blueClimbHighY);
            }

            good = good && (!inClimb);

            if (inZone) {
                good = good && (dist < ShooterTreeMap.getInstance().getMaxDistance());
            }

            double a = RobotState.getInstance().getPigeonAcceleration();

            if(RobotState.getInstance().getShuttling()) {
                good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.shuttlingMaxSpeed) && (RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond < SubsystemConstants.shuttlingMaxAngularVelocity);
            }
            else {
                double targetDist = RobotState.getInstance().getTargetPose().getTranslation().getDistance(RobotState.getInstance().getVisionPose().getTranslation());
                if (targetDist < ShooterTreeMap.getInstance().getMinDistance()) {
                    good = good && (RobotState.getInstance().getSpeed() < 0.1)
                            && (RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond < Math.PI/6.0);
                }
                else if (targetDist < 3.0) {
                    good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.autoAimMaxSpeed)
                            && (RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond < SubsystemConstants.autoAimMaxAngularVelocity);
                } else {
                    good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.autoAimMaxSpeedStrict)
                            && (RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond < SubsystemConstants.autoAimMaxAngularVelocityStrict);

                }
            }

            Logger.recordOutput("Indexer/autoIndex/nearTrenchBump", ((nearBlueTrench || nearRedTrench) && onBumpTrench));
            Logger.recordOutput("Indexer/autoIndex/inZone", inZone);
            Logger.recordOutput("Indexer/autoIndex/inClimb", inClimb);
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
                return SubsystemConstants.indexerShootVoltage;
            } else {
                return 0.0;
            }
        });
   }
}
