package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.hood.HoodSubsystem;
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


    public static Command turretAutoAim(RobotContainer robotContainer) {
        return robotContainer.getTurret().setPositionFieldRelative(
                () -> {
                    return  RobotState.getInstance().getAngleToVirtualTarget();
                }
        );
    }

    public static Command hoodAutoAim(RobotContainer robotContainer) {
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

            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(dist);
//            double pos = ShooterTreeMap.getInstance().calculateHoodAngle(0.0, dist);
//            Logger.recordOutput("Hood/autoAimAngle", pos);
//
//            return HoodSubsystem.toRotations(pos);

        });
    }

    public static Command shooterAutoSpinUp(RobotContainer robotContainer) {
        return robotContainer.getShooter().setSpeed(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
//            return ShooterTreeMap.getInstance().calculateRps(0.0, dist);
        });
    }

    public static Command autoAimCommand(RobotContainer robotContainer, Supplier<Pose2d> targetPoseSupplier, Supplier<ChassisSpeeds> unlockedSpeeds, boolean useTurret) {
        if (!useTurret) {
            return shooterAutoSpinUp(robotContainer)
                .alongWith(driveAutoAim(robotContainer, targetPoseSupplier, unlockedSpeeds))
                .alongWith(hoodAutoAim(robotContainer));
        } else {
             return shooterAutoSpinUp(robotContainer)
                .alongWith(turretAutoAim(robotContainer))
                .alongWith(hoodAutoAim(robotContainer));
        }
    }

    public static Command autoAimWhileMovingCommand(RobotContainer robotContainer, ShooterTreeMap treeMap, Supplier<ChassisSpeeds> unlockedSpeeds, boolean useTurret) {
        return autoAimCommand(
                robotContainer,
                () -> RobotState.getInstance().getShootingWhileMovingVirtualPose(),
                unlockedSpeeds,
                useTurret
        );
    }


   public static Command autoIndex(RobotContainer robotContainer) {
        BooleanSupplier readySupplier = () -> {
            boolean good;
            if(!SubsystemConstants.hasTurret) {
                good = RobotState.getInstance().getShooterReady() && RobotState.getInstance().getDriveShotReady();
            } else {
                good = RobotState.getInstance().getShooterReady() && robotContainer.getTurret().getShotReady();
            }

            if(DriverStation.isAutonomous()) good = good && RobotState.getInstance().getDriveAutoAiming();
            return good;
        };

        return robotContainer.getIndexer().setVoltage(() -> {
            Logger.recordOutput("Indexer/autoIndexShooterReady", RobotState.getInstance().getShooterReady());
            Logger.recordOutput("Indexer/autoIndexDriveReady", RobotState.getInstance().getDriveShotReady());

            if(readySupplier.getAsBoolean()) {
                Logger.recordOutput("Indexer/ready", true);
                return -9.0;
            } else {
                Logger.recordOutput("Indexer/ready", false);
                return 0.0;
            }
        }, () -> {
            if (readySupplier.getAsBoolean()) {
                return -12.0;
            }
            else {
                return 0.0;
            }
        });/*.alongWith(robotContainer.getIntake().setRollerVoltage(() -> {
                    if (readySupplier.getAsBoolean()) {
                        return 4.5;
                    }
                    else {
                        return 0.0;
                    }
                }));*/
   }

    public static Command autoAimFlush(RobotContainer robotContainer) {
        return robotContainer.getHood().setPosition(() -> 0.0)
                .alongWith(robotContainer.getShooter().setSpeed(() -> ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(0.5)));
    }

    public static Command shootFlush(RobotContainer robotContainer) {
        return robotContainer.getHood().setPosition(() -> 0.0)
                .alongWith(robotContainer.getShooter().setSpeed(() -> ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(0.5)))
                .alongWith(robotContainer.getIndexer().setVoltage(() -> {
                    boolean good = RobotState.getInstance().getShooterReady() && (RobotState.getInstance().getSpeed() < 0.1);
                    if (good) {
                        return -9.0;
                    }
                    else {
                        return 0.0;
                    }
                }, () -> {
                    boolean good = RobotState.getInstance().getShooterReady() && (RobotState.getInstance().getSpeed() < 0.1);
                    if (good) {
                        return -12.0;
                    }
                    else {
                        return 0.0;
                    }
                }
            )
        );
    }

//    public static Command shooterAutoSpinUp(RobotContainer robotContainer) {
//        return robotContainer.getShooter().setTargetExitVelocity(() -> {
//            Translation3d vec = RobotState.getInstance().getShotVectorWhileMoving();
//            return vec.getNorm();
//        });
//    }
//
//    public static Command hoodAutoAim(RobotContainer robotContainer) {
//        return robotContainer.getHood().setPositionRadians(() -> {
//            Translation3d vec = RobotState.getInstance().getShotVectorWhileMoving();
//            double up = vec.getZ();
//            double forward = Math.hypot(vec.getX(), vec.getY());
//
//            return Math.atan2(up, forward);
//        });
//    }
//
//    public static Command turretAutoAim(RobotContainer robotContainer) {
//        return robotContainer.getTurret().setPositionFieldRelative(() -> {
//            Translation3d vec = RobotState.getInstance().getShotVectorWhileMoving();
//            return Math.atan2(vec.getY(), vec.getX());
//        });
//    }
}
