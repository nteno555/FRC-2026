package frc2025.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.ShooterTreeMap;
import org.littletonrobotics.junction.Logger;

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

    public static Command turretAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return robotContainer.getTurret().setPositionFieldRelative(
                () -> {
                    return RobotState.getInstance().getAngleToVirtualTarget();
                }
        );
    }
    
    public static double clampTurretAngle(double wantedAngle, double currentAngle) {
        double min_angle = Math.PI * -2;
        double max_angle = Math.PI * 2;

        double[] candidates = {wantedAngle, wantedAngle + 2*Math.PI, wantedAngle - 2*Math.PI};
    
        double bestAngle = currentAngle;  
        double minDistance = 9999999.0;
        
        for (double candidate : candidates) {
            if (candidate >= min_angle && candidate <= max_angle) {
                double distance = Math.abs(candidate - currentAngle);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestAngle = candidate;
                }
            }
        }

        return bestAngle;

    }

    public static Command pivotAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier) {
        return robotContainer.getPivot().setPosition(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            Pose2d targetPose = targetPoseSupplier.get();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().pivotTreeMap.getInterpolated(dist);
        });
    }

    public static Command shooterAutoSpinUp(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier) {
        return robotContainer.getShooter().setSpeed(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            Pose2d targetPose = targetPoseSupplier.get();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
        });
    }

    public static Command autoAimCommand(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds, boolean useTurret) {
        if(useTurret) {
            return shooterAutoSpinUp(robotContainer, targetPoseSupplier)
                    .alongWith(turretAutoAim(robotContainer, targetPoseSupplier, unlockedSpeeds))
                    .alongWith(pivotAutoAim(robotContainer, targetPoseSupplier));
        } else {
             return shooterAutoSpinUp(robotContainer, targetPoseSupplier)
                    .alongWith(turretAutoAim(robotContainer, targetPoseSupplier, unlockedSpeeds))
                    .alongWith(pivotAutoAim(robotContainer, targetPoseSupplier));
        }

    }

    // calculate the virtual pose of the hub
//    public static Pose2d calculateShootingWhileMoving(RobotContainer robotContainer, ShooterTreeMap treeMap, Pose2d realTargetPose) {
//        Pose2d currentPose = RobotState.getInstance().getVisionPose();
//        //Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
//        Pose2d virtualPose = realTargetPose;
//
//        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
//            robotContainer.getDrive().getRobotRelativeSpeeds(),
//            currentPose.getRotation()
//        );
//        Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
//
//        for (int i = 0; i < 5; i++) {
//            double dx = virtualPose.getX() - currentPose.getX();
//            double dy = virtualPose.getY() - currentPose.getY();
//            double distance = Math.hypot(dx, dy);
//
//            double flightTime = treeMap.tofTreeMap.getInterpolated(distance);
//
//            virtualPose = new Pose2d(
//                realTargetPose.getX() + velocity.getX() * flightTime,
//                realTargetPose.getY() + velocity.getY() * flightTime,
//                realTargetPose.getRotation()
//            );
//        }
//
//        return virtualPose;
//    }

    public static Command autoAimWhileMovingCommand(RobotContainer robotContainer, ShooterTreeMap treeMap, Supplier<ChassisSpeeds> unlockedSpeeds, boolean useTurret) {
        return autoAimCommand(
                robotContainer,
                () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                unlockedSpeeds,
                useTurret
        );
    }


    public static Command autoIndex(RobotContainer robotContainer) {
        Supplier<Pair<Double, Double>> voltageSupplier = () -> {
            if (robotContainer.getShooter().getShotReady() && robotContainer.getDrive().getShotReady()) {
                return new Pair<>(-3.0, -3.0);
            } else {
                return new Pair<>(0.0, 0.0);
            }
        };

        return robotContainer.getIndexer().setSpeed(() -> voltageSupplier.get().getFirst(), () -> voltageSupplier.get().getSecond());
    }
}
