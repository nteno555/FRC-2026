package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.subsystems.shooter.ShooterSubsystem;
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

    public static Command autoShoot(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds, Supplier<Boolean> hasTurret) {
        return (hasTurret.get()) ?
                turretAutoAim(robotContainer) :
                driveAutoAim(robotContainer, targetPoseSupplier, unlockedSpeeds)
                        .alongWith(robotContainer.getHood().autoAim())
                        .alongWith(robotContainer.getShooter().autoAim())
                        .alongWith(autoIndex(robotContainer));
    }


    public static Command autoShoot(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return (SubsystemConstants.hasTurret) ?
                turretAutoAim(robotContainer) :
                driveAutoAim(robotContainer, targetPoseSupplier, unlockedSpeeds)
                        .alongWith(robotContainer.getHood().autoAim())
                        .alongWith(robotContainer.getShooter().autoAim())
                        .alongWith(autoIndex(robotContainer));
    }

    public static Command turretAutoAim(RobotContainer robotContainer) {
        return robotContainer.getTurret().autoAim()
                .alongWith(robotContainer.getHood().autoAim())
                .alongWith(robotContainer.getShooter().autoAim())
//                .alongWith(robotContainer.getIntake().setDeployPosition(() -> SubsystemConstants.intakeShotPosition))
                .alongWith(autoIndex(robotContainer));
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
                                    angularVelocity
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

            boolean nearBlueTrench = Math.abs(pose.getX() - FieldConstants.blueTrenchX) < FieldConstants.trenchThreshold;
            boolean nearRedTrench = Math.abs(pose.getX() - FieldConstants.redTrenchX) < FieldConstants.trenchThreshold;

            if ((nearBlueTrench || nearRedTrench)) return false;

            boolean inBlueZone = pose.getX() < FieldConstants.inBlueZoneX && !Alliance.get();
            boolean inRedZone = pose.getX() > FieldConstants.inRedZoneX && Alliance.get();

            if (inBlueZone || inRedZone) {
                good = good && (dist < ShooterTreeMap.getInstance().getMaxDistance());
            }

            if(DriverStation.isAutonomous()) {
                good = good && (inBlueZone || inRedZone);
            }

            double a = RobotState.getInstance().getPigeonAcceleration();

            if(RobotState.getInstance().getShuttling()) {
                good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.shuttlingMaxSpeed) && (RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond < SubsystemConstants.shuttlingMaxAngularVelocity);
            } else {
                double targetDist = RobotState.getInstance().getTargetPose().getTranslation().getDistance(RobotState.getInstance().getVisionPose().getTranslation());

                if(targetDist < 2.7) {
                    good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.autoAimMaxSpeed)
                            && (RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond < SubsystemConstants.autoAimMaxAngularVelocity);
                } else {
                    good = good && (RobotState.getInstance().getSpeed() < SubsystemConstants.autoAimMaxSpeedStrict)
                            && (RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond < SubsystemConstants.autoAimMaxAngularVelocityStrict);

                }
            }

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
        });
   }

//   public static Command autoAimVector(RobotContainer robotContainer, Supplier<ChassisSpeeds> unlockedSpeeds) {
//        Supplier<Translation3d> shotVectorSupplier = () -> {
//            Pose2d currentPose = RobotState.getInstance().getVisionPose();
//            Pose2d targetPose = RobotState.getInstance().getTargetPose();
//            double phaseDelay = 0.03;
//
//            ChassisSpeeds chassisSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
//            double vx = chassisSpeeds.vxMetersPerSecond;
//            double vy = chassisSpeeds.vyMetersPerSecond;
//            double vOmega = chassisSpeeds.omegaRadiansPerSecond;
//            Pose2d delayedPose = currentPose.exp(
//                    new Twist2d(vx * phaseDelay, vy * phaseDelay, vOmega * phaseDelay)
//            );
//
//            double dist = delayedPose.getTranslation().getDistance(targetPose.getTranslation());
//
//            double flywheelVelocity = ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
//            double hoodAngle = ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(dist);
//            double turretAngle = Math.atan2(targetPose.getY() - delayedPose.getY(), targetPose.getX() - delayedPose.getX());
//
//            double exitVelocity = ShooterSubsystem.toBallExitVelocity(flywheelVelocity);
//            double realHoodAngle = HoodSubsystem.toRadians(hoodAngle);
//
//            Translation3d shotVector = new Translation3d(
//                    exitVelocity * Math.cos(realHoodAngle) * Math.cos(turretAngle),
//                    exitVelocity * Math.cos(realHoodAngle) * Math.sin(turretAngle),
//                    exitVelocity * Math.sin(realHoodAngle)
//            );
//
//            Translation3d velocityVector = new Translation3d(vx, vy, 0);
//
//            Logger.recordOutput("ShootWhileMoving/shotVector", shotVector);
//            Logger.recordOutput("ShootWhileMoving/velocityVector", velocityVector);
//
//            return shotVector.plus(velocityVector);
//        };
//
//        if(SubsystemConstants.hasTurret) {
//            return robotContainer.getShooter().setTargetExitVelocity(() -> shotVectorSupplier.get().getNorm())
//                    .alongWith(
//                        robotContainer.getHood().setPositionRadians(
//                                () -> Math.atan2(shotVectorSupplier.get().getZ(), Math.hypot(shotVectorSupplier.get().getX(), shotVectorSupplier.get().getY())))
//                    ).alongWith(
//                            robotContainer.getTurret().setPositionFieldRelative(() -> Math.atan2(shotVectorSupplier.get().getY(), shotVectorSupplier.get().getX()))
//                    );
//        } else {
//            return robotContainer.getShooter().setTargetExitVelocity(() -> shotVectorSupplier.get().getNorm())
//                    .alongWith(
//                            robotContainer.getHood().setPositionRadians(
//                                    () -> Math.atan2(shotVectorSupplier.get().getZ(), Math.hypot(shotVectorSupplier.get().getX(), shotVectorSupplier.get().getY())))
//                    ).alongWith(
//                            robotContainer.getDrive().driveFieldRelative(
//                                    () -> {
//                                        PIDController angleController = new PIDController(5, 0.0, 0.0);
//                                        angleController.enableContinuousInput(-Math.PI, Math.PI);
//
//                                        Pose2d shooterPose = RobotState.getInstance().getVisionShooterPose();
//                                        double turretAngle = Math.atan2(shotVectorSupplier.get().getY(), shotVectorSupplier.get().getX());
//                                        double angleToTarget = turretAngle;
//
//                                        Logger.recordOutput("Drive/hubTargetPose", RobotState.getInstance().getTargetPose());
//                                        Logger.recordOutput("Drive/robotRotation", modAngle(shooterPose.getRotation().getRadians()));
//                                        Logger.recordOutput("Drive/robotRotationUnmodded", shooterPose.getRotation().getRadians());
//
//                                        return new ChassisSpeeds(
//                                                unlockedSpeeds.get().vxMetersPerSecond,
//                                                unlockedSpeeds.get().vyMetersPerSecond,
//                                                -angleController.calculate(modAngle(shooterPose.getRotation().getRadians()), modAngle(angleToTarget))
//                                        );
//                                    }
//                            )
//                    );
//        }
//   }
}
