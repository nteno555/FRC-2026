package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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

                    /*double angleToTarget = Math.atan2(
                            targetPose.getY() - shooterPose.getY(),
                            targetPose.getX() - shooterPose.getX()
                    );*/
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

    public static Command hoodAutoAim(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier) {
        return robotContainer.getHood().setPosition(() -> {
            
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            double robotXpose = robotPose.getX();
            double robotYpose = robotPose.getY();
            
            boolean nearBlueUpperTrench = (robotXpose < 6.25 && robotXpose > 3.0) && robotYpose > 7.0; 
            boolean nearBlueLowerTrench = (robotXpose < 6.25 && robotXpose > 3.0) && robotYpose < 1.2; 
            boolean nearRedUpperTrench = (robotXpose < 13.5 && robotXpose > 10.25) && robotYpose > 7.0; 
            boolean nearRedLowerTrench = (robotXpose < 13.5 && robotXpose > 10.25) && robotYpose < 1.2; 
            
            if (nearBlueUpperTrench || nearBlueLowerTrench || nearRedUpperTrench || nearRedLowerTrench) {
                return 0.0;
            }
            
            Pose2d targetPose = targetPoseSupplier.get();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(dist);
            
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

    public static Command autoAimCommand(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return shooterAutoSpinUp(robotContainer, targetPoseSupplier)
                .alongWith(driveAutoAim(robotContainer, targetPoseSupplier, unlockedSpeeds))
                .alongWith(hoodAutoAim(robotContainer, targetPoseSupplier));
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

    public static Command autoAimWhileMovingCommand(RobotContainer robotContainer, ShooterTreeMap treeMap, Supplier<ChassisSpeeds> unlockedSpeeds) {
        return autoAimCommand(
                robotContainer,
                () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                unlockedSpeeds);
    }


    public static Command autoIndex(RobotContainer robotContainer) {
        return robotContainer.getIndexer().setVoltage(() -> {
//            Logger.recordOutput("Indexer/autoIndexShooterReady", robotContainer.getShooter().getShotReady());
//            Logger.recordOutput("Indexer/autoIndexDriveReady", robotContainer.getDrive().getShotReady());
            Logger.recordOutput("Indexer/autoIndexShooterReady", RobotState.getInstance().getShooterReady());
            Logger.recordOutput("Indexer/autoIndexDriveReady", RobotState.getInstance().getDriveShotReady());
//            if(robotContainer.getShooter().getShotReady() && robotContainer.getDrive().getShotReady()) {
            boolean good = RobotState.getInstance().getShooterReady() && RobotState.getInstance().getDriveShotReady();
            if(DriverStation.isAutonomous()) good = good && RobotState.getInstance().getDriveAutoAiming();

            if(good) {
                Logger.recordOutput("Indexer/ready", true);
                return -9.0;
            } else {
                Logger.recordOutput("Indexer/ready", false);
                return 0.0;
            }
        }, () -> {
            boolean good = RobotState.getInstance().getShooterReady() && RobotState.getInstance().getDriveShotReady();
            if(DriverStation.isAutonomous()) good = good && RobotState.getInstance().getDriveAutoAiming();

            if (good) {
                return -12.0;
            }
            else {
                return 0.0;
            }
        });
    }
}
