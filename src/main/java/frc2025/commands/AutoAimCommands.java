package frc2025.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.util.Alliance;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc2025.RobotContainer.DEAD_BAND_DRIVER;
import static frc2025.RobotContainer.handleDeadBand;
import static frc2025.subsystems.drive.DriveConstants.kSpeedAt12Volts;

public class AutoAimCommands {
    private static double modAngle(double angle) {
        if(angle > Math.PI) return angle - 2*Math.PI;
        return angle;
    }

    public static Command driveAutoAim(RobotContainer robotContainer, boolean useDriver) {
        PIDController angleController = new PIDController(1, 0.0, 0.0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return robotContainer.getDrive().driveFieldRelative(
                () -> {
                    Pose2d robotPose = RobotState.getInstance().getVisionPose();
                    Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
                    double angleToTarget = Math.atan2(
                            targetPose.getY() - robotPose.getY(),
                            targetPose.getX() - robotPose.getX()
                    );
                    double angularVelocity = angleController.calculate(modAngle(robotPose.getRotation().getRadians()), modAngle(angleToTarget));

                    if(useDriver) {
                        return new ChassisSpeeds(
                                handleDeadBand(robotContainer.getDriverLeftY(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                handleDeadBand(robotContainer.getDriverLeftX(), DEAD_BAND_DRIVER, true) * kSpeedAt12Volts.in(MetersPerSecond),
                                angularVelocity
                        );
                    } else {
                        return new ChassisSpeeds(
                                0,
                                0,
                                angularVelocity
                        );
                    }
                }
        );
    }

    public static Command pivotAutoAim(RobotContainer robotContainer) {
        return robotContainer.getPivot().setPosition(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionPose();
            Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().pivotTreeMap.getInterpolated(dist);
        });
    }

    public static Command shooterAutoSpinUp(RobotContainer robotContainer) {
        return robotContainer.getShooter().setSpeed(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionPose();
            Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
        });
    }

    public static Command autoAimCommand(RobotContainer robotContainer, boolean useDriver) {
        return shooterAutoSpinUp(robotContainer)
                .alongWith(driveAutoAim(robotContainer, useDriver))
                .alongWith(pivotAutoAim(robotContainer));
    }

    public static double shootingWhileMoving(RobotContainer robotContainer, ShooterTreeMap treeMap) {
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        
        double dx = FieldConstants.goalPose.getX() - currentPose.getX();
        double dy = FieldConstants.goalPose.getY() - currentPose.getY();
        double currentDistance = Math.hypot(dx, dy);  
        

        ChassisSpeeds velocity = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotContainer.getDrive().getRobotRelativeSpeeds(),
            currentPose.getRotation()
        );
        
        double angleToTarget = Math.atan2(dy, dx);
        

        double vx = velocity.vxMetersPerSecond;
        double vy = velocity.vyMetersPerSecond;
        double velocityTowardsGoal = vx * Math.cos(angleToTarget) + vy * Math.sin(angleToTarget);

        double effectiveDistance = currentDistance;
        for (int i = 0; i < 5; i++) {
            double flightTime = treeMap.tofTreeMap.getInterpolated(effectiveDistance);
            double movement = velocityTowardsGoal * flightTime;
            effectiveDistance = currentDistance + movement;
        }
        
        return effectiveDistance;
    }

    public static Command autoAimWhileMovingCommand(RobotContainer robotContainer, ShooterTreeMap treeMap) {
        return driveAutoAim(robotContainer, false).alongWith(
            robotContainer.getPivot().setPosition(() -> {
                double effectiveDistance = shootingWhileMoving(robotContainer, treeMap);
                return treeMap.pivotTreeMap.getInterpolated(effectiveDistance);
            }).alongWith(
                robotContainer.getShooter().setSpeed(() -> {
                    double effectiveDistance = shootingWhileMoving(robotContainer, treeMap);
                    return treeMap.upperWheelTreeMap.getInterpolated(effectiveDistance);
                })
            )
        );
    }
}
