package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
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

import java.sql.Driver;
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
//        Debouncer debouncer = new Debouncer(SubsystemConstants.autoIndexDebounceTime);
        BooleanSupplier readySupplier = () -> {
            boolean good = robotContainer.getShooter().getShotReady() && robotContainer.getHood().getShotReady();
            if (!SubsystemConstants.hasTurret) {
                good = good && robotContainer.getDrive().getShotReady();
            } else {
                good = good && robotContainer.getTurret().getShotReady();
            }

            Pose2d pose = RobotState.getInstance().getVisionPose();

//            boolean inBlueZone = pose.getX() < FieldConstants.inBlueZoneX && !Alliance.get();
//            boolean inRedZone = pose.getX() > FieldConstants.inRedZoneX && Alliance.get();
//            good = good && (inBlueZone || inRedZone);

            // boolean debouncedGood = debouncer.calculate(good);

            Logger.recordOutput("Indexer/autoIndex/shooterReady", robotContainer.getShooter().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/turretReady", robotContainer.getTurret().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/hoodReady", robotContainer.getHood().getShotReady());
//            Logger.recordOutput("Indexer/autoIndex/speed", RobotState.getInstance().getSpeed());
            Logger.recordOutput("Indexer/autoIndexReady", good);

            return good;
        };

        return robotContainer.getIndexer().setIndexerVoltageWithTowerSpeed(() -> {
            if (readySupplier.getAsBoolean()) {
                return SubsystemConstants.indexerShootVoltage;
            } else {
                return SubsystemConstants.indexerReverseVoltage;
            }
        }, () -> {
            if (readySupplier.getAsBoolean()) {
                double indexerSpeed = robotContainer.getShooter().getVelocity();
                return indexerSpeed;
            } else {
                return 0.0;
            }
        });
    }

    public static Command autoIndexTeleop(RobotContainer robotContainer, Supplier<Boolean> hasTurret) {
        Debouncer debouncer = new Debouncer(SubsystemConstants.autoIndexDebounceTime);
        BooleanSupplier readySupplier = () -> {
            boolean good = robotContainer.getShooter().getShotReady() && robotContainer.getHood().getShotReady();

            if (!hasTurret.get()) {
                good = good && robotContainer.getDrive().getShotReady();
            } else {
                good = good && robotContainer.getTurret().getShotReady();
            }

            if (RobotState.getInstance().getShuttling()) {
                boolean slowEnough = RobotState.getInstance().getSpeed() < 3.0;
                boolean xVelocityReady;

                DriverStation.Alliance alliance = DriverStation.Alliance.Red;
                if(DriverStation.getAlliance().isPresent()) {
                    alliance = DriverStation.getAlliance().get();
                }

                if(alliance == DriverStation.Alliance.Red) {
                    xVelocityReady = RobotState.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond > -3.0;
                } else {
                    xVelocityReady = RobotState.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond < 3.0;
                }

                Logger.recordOutput("Indexer/autoIndex/turretReady", robotContainer.getTurret().getShotReadyLenient());
                Logger.recordOutput("Indexer/autoIndex/hoodReady", robotContainer.getHood().getShotReadyLenient());
                Logger.recordOutput("Indexer/autoIndex/xVelocityReady", xVelocityReady);
                Logger.recordOutput("Indexer/autoIndex/xVelocity", RobotState.getInstance().getFieldRelativeSpeeds().vxMetersPerSecond);

                return robotContainer.getTurret().getShotReadyLenient() && robotContainer.getHood().getShotReadyLenient()
                        && slowEnough && xVelocityReady;
                // return true;
                // return debouncer.calculate(robotContainer.getTurret().getShotReady() &&
                // robotContainer.getHood().getShotReady());
            }

            Pose2d pose = RobotState.getInstance().getVisionPose();
            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = pose.getTranslation().getDistance(targetPose.getTranslation());
            double xPos = pose.getX();
            double yPos = pose.getY();

            double xySpeed = RobotState.getInstance().getSpeed();
            double omegaSpeed = RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond;

            boolean nearBlueTrench = Math.abs(xPos - FieldConstants.blueTrenchX) < FieldConstants.trenchThreshold;
            boolean nearRedTrench = Math.abs(xPos - FieldConstants.redTrenchX) < FieldConstants.trenchThreshold;
            boolean onBumpTrench = (yPos > FieldConstants.highBumpY) || (yPos < FieldConstants.lowBumpY);

            boolean inZone;
            boolean inClimb;

            // Pose Checks
            good = good && !((nearBlueTrench || nearRedTrench) && onBumpTrench);

            if (Alliance.get()) {
                inZone = xPos > FieldConstants.inRedZoneX;
//                inClimb = (xPos > FieldConstants.redClimbX) && (yPos > FieldConstants.redClimbLowY)
//                        && (yPos < FieldConstants.redClimbHighY);
            } else {
                inZone = xPos < FieldConstants.inBlueZoneX;
//                inClimb = (xPos < FieldConstants.blueClimbX) && (yPos > FieldConstants.blueClimbLowY)
//                        && (yPos < FieldConstants.blueClimbHighY);
            }

//            good = good && (!inClimb);

            if (inZone) {
                good = good && (dist < ShooterTreeMap.getInstance().getMaxDistance());
            }
            // double a = RobotState.getInstance().getPigeonAcceleration();

            // Speed Checks
            if (dist < ShooterTreeMap.getInstance().getMinDistance()) {
                good = good && (xySpeed < 0.1) && (omegaSpeed < Math.PI / 6.0);
            } else if (dist < 3.0) {
                good = good && (xySpeed < SubsystemConstants.autoAimMaxSpeed)
                        && (omegaSpeed < SubsystemConstants.autoAimMaxAngularVelocity);
            } else {
                good = good && (xySpeed < SubsystemConstants.autoAimMaxSpeedStrict)
                        && (omegaSpeed < SubsystemConstants.autoAimMaxAngularVelocityStrict);

            }

            Logger.recordOutput("Indexer/autoIndex/shooterReady", robotContainer.getShooter().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/turretReady", robotContainer.getTurret().getShotReady());
            Logger.recordOutput("Indexer/autoIndex/hoodReady", robotContainer.getHood().getShotReady());
            Logger.recordOutput("Indexer/autoIndexReady", good);
            Logger.recordOutput("Indexer/autoIndexReady", good);

            return good;
        };

        return robotContainer.getIndexer().setIndexerVoltageWithTowerSpeed(() -> {
            if (readySupplier.getAsBoolean()) {
                return SubsystemConstants.indexerShootVoltage;
            } else {
                return 0.0;
            }
        }, () -> {
            if (readySupplier.getAsBoolean()) {
                return robotContainer.getShooter().getVelocity();
            } else {
                return 0.0;
            }
        });
    }
}
