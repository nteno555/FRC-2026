package frc2025.subsystems.turret;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
    TurretIO io;
    TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    double setpoint, voltage;
    boolean zeroed = false;

    public TurretSubsystem(TurretIO io) {
        this.io = io;
        io.rezero();
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Turret", inputs);
        Logger.recordOutput("Turret/setpoint", setpoint);

        io.updateInputs(inputs);

        double fieldRelativeAngleRad = -inputs.positionRadians
                + RobotState.getInstance().getVisionPose().getRotation().getRadians();
        Logger.recordOutput("Turret/fieldRelativeAngleRadians", fieldRelativeAngleRad);

        RobotState.getInstance().setTurretAngle(inputs.positionRadians);
        RobotState.getInstance().setTurretVelocity(inputs.velocity);

//        RobotState.getInstance().addTimestampedTurretAngleRadians(inputs.positionRadians, Utils.getCurrentTimeSeconds());

        if (SubsystemConstants.hasTurret) {
            if (!zeroed && inputs.throughboresConnected) {
                io.rezeroCRT();
                zeroed = true;
            }
        } else {
            if (!zeroed) {
                io.rezero(0.0);
                zeroed = true;
            }
        }
//
//
//        Pose2d turretPose = new Pose2d(
//                RobotState.getInstance().getVisionShooterPose().getTranslation(),
//                Rotation2d.fromRadians(fieldRelativeAngleRad));
//        Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
//        Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
//        double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
//        Transform2d aimingPoseTransform = new Transform2d(dist, 0.0, Rotation2d.kZero);
//        Pose2d aimingPose = turretPose.transformBy(aimingPoseTransform);
//
//        Logger.recordOutput("Turret/turretPose", turretPose);
//        Logger.recordOutput("Turret/turretAimingPose", new Pose2d(aimingPose.getTranslation(), Rotation2d.kZero));
    }

    public Command autoAim() {
        return setPositionFieldRelative(
                () -> {
                    double wantedAngle = RobotState.getInstance().getAngleToVirtualTarget();
                    return wantedAngle;
                },
                () -> {
                    // ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
                    // double vx = robotSpeeds.vxMetersPerSecond;
                    // double vy = robotSpeeds.vyMetersPerSecond;
                    // double vOmega = robotSpeeds.omegaRadiansPerSecond;

                    // Translation2d targetPos = RobotState.getInstance().getShootingWhileMovingVirtualPose()
                    //         .getTranslation();
                    // var robotPose = RobotState.getInstance().getDelayedPose();
                    // Translation2d shooterPos = robotPose.transformBy(SubsystemConstants.robotToShooter)
                    //         .getTranslation();

                    // var robotToTargetXY = targetPos.minus(shooterPos);
                    // Translation2d shooterRobotRelative = SubsystemConstants.robotToShooter.getTranslation()
                    //         .rotateBy(robotPose.getRotation());

                    // robotSpeeds = new ChassisSpeeds(
                    //         vx + vOmega * shooterRobotRelative.getY(),
                    //         vy - vOmega * shooterRobotRelative.getX(),
                    //         vOmega);

                    // var targetFrameToRobot = new Translation2d(robotSpeeds.vxMetersPerSecond,
                    //         robotSpeeds.vyMetersPerSecond)
                    //         .rotateBy(robotToTargetXY.getAngle());

                    // var tangent = targetFrameToRobot.getY();
                    // var angular = robotSpeeds.omegaRadiansPerSecond;
                    // var distanceToTarget = robotToTargetXY.getNorm();

                    // return (angular + tangent / distanceToTarget);
                    // return RobotState.getInstance().getVirtualTargetAngularVelocity();

                    return RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond 
                           - RobotState.getInstance().getVirtualTargetAngularVelocity();

                },
                () -> false);
    }

    public Command snapThenTrack() {
        return setPositionFieldRelative(() -> {
            double wantedAngle = RobotState.getInstance().getAngleToVirtualTarget();
            return wantedAngle;
        }, () -> 0.0, () -> true).until(this::getShotReady)
                .andThen(autoAim());
    }

    public Command setPositionFieldRelative(Supplier<Double> positionRadians, Supplier<Double> velocityRadiansPerSecond,
            Supplier<Boolean> fast) {
        return setPositionSafe(
                () -> -(positionRadians.get() - RobotState.getInstance().getVisionPose().getRotation().getRadians()),
                velocityRadiansPerSecond, fast);
    }

    public Command setPositionSafe(Supplier<Double> positionRadians, Supplier<Double> velocityRadiansPerSecond,
            Supplier<Boolean> fast) {
        return this.run(() -> {
            double minAngle = SubsystemConstants.turretMinAngleRadians;
            double maxAngle = SubsystemConstants.turretMaxAngleRadians;

            double[] candidates = { positionRadians.get(), positionRadians.get() + 2 * Math.PI,
                    positionRadians.get() - 2 * Math.PI };

            double bestAngle = positionRadians.get();
            double minDistance = 9999999.0;

            for (double candidate : candidates) {
                if (candidate >= minAngle && candidate <= maxAngle) {
                    double distance = Math.abs(candidate - inputs.positionRadians);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestAngle = candidate;
                    }
                }
            }

            if (bestAngle < minAngle) {
                bestAngle = minAngle;
            } else if (bestAngle > maxAngle) {
                bestAngle = maxAngle;
            }

            double velocityFF = velocityRadiansPerSecond.get() * SubsystemConstants.turretRotationsPerRadian;

            if (Math.abs(bestAngle - inputs.positionRadians) > Math.toRadians(180.0)) {
                velocityFF = 0;
            }

            setpoint = io.toRotations(bestAngle);

            if (!fast.get()) {
                io.setPosition(setpoint, velocityFF);
            } else {
                io.setPositionFast(setpoint);
            }

//            Logger.recordOutput("Turret/turretWantedPositionRadians", positionRadians.get());
//            Logger.recordOutput("Turret/turretClampedPositionRadians", bestAngle);
//            Logger.recordOutput("Turret/turretClampedPositionRotations", setpoint);
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            this.voltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command rezeroCRT() {
        return this.runOnce(() -> io.rezeroCRT());
    }

    public Command rezero() {
        return this.runOnce(() -> io.rezero());
    }

    public Command homeThenRezero() {
        return this.runEnd(() -> {
            io.setVoltage(-2.0);
            io.rezero(SubsystemConstants.turretHomeAngleRotations);
        }, () -> io.setVoltage(0.0));
    }

    public Command aimAtHub() {
        return setPositionFieldRelative(() -> RobotState.getInstance().getAngleToRealHub(),
                () -> 0.0, () -> true).until(this::getShotReady)
            .andThen(setPositionFieldRelative(() ->
                RobotState.getInstance().getAngleToRealHub(),
            () -> RobotState.getInstance().getFieldRelativeSpeeds().omegaRadiansPerSecond - RobotState.getInstance().getVirtualTargetAngularVelocity(),
            () -> false
        ));
    }

    public double getPosition() {
        return inputs.positionTicks;
    }

    public double getPositionRadians() {
        return inputs.positionRadians;
    }

    public boolean equals(double a, double b) {
        double epsilon = 0.001;
        return Math.abs(a - b) < epsilon;
    }

    public boolean getShotReady() {
        Logger.recordOutput("Turret/errorRads", Math.abs(inputs.positionRadians - io.toRadians(setpoint)));
        return Math.abs(inputs.positionRadians - io.toRadians(setpoint)) < SubsystemConstants.turretShotReadyThreshold
                && !(equals(setpoint, io.toRotations(SubsystemConstants.turretMinAngleRadians))
                        || equals(setpoint, io.toRotations(SubsystemConstants.turretMaxAngleRadians)));
    }

    public boolean getShotReadyLenient() {
        return Math.abs(inputs.positionRadians - io.toRadians(setpoint)) < 0.35
                && !(equals(setpoint, io.toRotations(SubsystemConstants.turretMinAngleRadians))
                || equals(setpoint, io.toRotations(SubsystemConstants.turretMaxAngleRadians)));
    }
}