package frc2025.subsystems.turret;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        io.rezero(0.0);
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Turret/currentCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        io.updateInputs(inputs);
        Logger.processInputs("RealOutputs/Turret", inputs);
        Logger.recordOutput("Turret/setpoint", setpoint);

        double fieldRelativeAngleRad = -inputs.positionRadians
                + RobotState.getInstance().getDelayedPose().getRotation().getRadians();
        Logger.recordOutput("Turret/fieldRelativeAngleRadians", fieldRelativeAngleRad);
        
        // Heading Error Logging
        var rs = RobotState.getInstance();
        double heading = rs.getDelayedPose().getRotation().getRadians();
        double fieldAngle = -inputs.positionRadians + heading;

        Logger.recordOutput("Turret/err/trackingDeg",
                Math.toDegrees(inputs.positionRadians - io.toRadians(setpoint)));
        Logger.recordOutput("Turret/err/aimVirtualDeg",
                Math.toDegrees(MathUtil.angleModulus(fieldAngle - rs.getAngleToVirtualTarget())));
        Logger.recordOutput("Turret/err/aimRealDeg",
                Math.toDegrees(MathUtil.angleModulus(fieldAngle - rs.getAngleToRealHub())));
        Logger.recordOutput("Turret/err/leadDeg",
                Math.toDegrees(MathUtil.angleModulus(rs.getAngleToVirtualTarget() - rs.getAngleToRealHub())));
        Logger.recordOutput("Turret/err/robotSpeed", rs.getSpeed());

        Pose2d shooterPose = rs.getDelayedPose().transformBy(SubsystemConstants.robotToShooter);
        Pose2d turretPose = new Pose2d(shooterPose.getTranslation(), Rotation2d.fromRadians(fieldAngle));
        Pose2d virtualPose = rs.getShootingWhileMovingVirtualPose();
        double dist = shooterPose.getTranslation().getDistance(virtualPose.getTranslation());
        Pose2d aimPoint = turretPose.transformBy(new Transform2d(dist, 0.0, Rotation2d.kZero));

        Logger.recordOutput("Turret/viz/turretPose", turretPose);
        Logger.recordOutput("Turret/viz/aimPoint", new Pose2d(aimPoint.getTranslation(), Rotation2d.kZero));
        Logger.recordOutput("Turret/viz/virtualTarget", new Pose2d(virtualPose.getTranslation(), Rotation2d.kZero));
        RobotState.getInstance().setTurretAngle(inputs.positionRadians);
        RobotState.getInstance().setTurretVelocity(inputs.velocity);

//        RobotState.getInstance().addTimestampedTurretAngleRadians(inputs.positionRadians, Utils.getCurrentTimeSeconds());

        // if (SubsystemConstants.hasTurret) {
        //     if (!zeroed && inputs.throughboresConnected) {
        //         io.rezeroCRT();
        //         zeroed = true;
        //     }
        // } else {
        // if (!zeroed) {
        //     io.rezero(0.0);
        //     zeroed = true;
        // }
    }
    

    public Command autoAim() {
        return setPositionFieldRelative(
                () -> {
                    double wantedAngle = RobotState.getInstance().getAngleToVirtualTarget();
                    return wantedAngle;
                },
                () -> {
                    ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
                    double vx = robotSpeeds.vxMetersPerSecond;
                    double vy = robotSpeeds.vyMetersPerSecond;
                    double vOmega = robotSpeeds.omegaRadiansPerSecond;

                    Translation2d targetPos = RobotState.getInstance().getShootingWhileMovingVirtualPose()
                            .getTranslation();
                    var robotPose = RobotState.getInstance().getDelayedPose();
                    Translation2d shooterPos = robotPose.transformBy(SubsystemConstants.robotToShooter)
                            .getTranslation();

                    var robotToTargetXY = targetPos.minus(shooterPos);

                    Translation2d shooterOffsetField = SubsystemConstants.robotToShooter.getTranslation()
                            .rotateBy(robotPose.getRotation());


                    double shooterVx = vx - vOmega * shooterOffsetField.getY();
                    double shooterVy = vy + vOmega * shooterOffsetField.getX();

                    double tangent = new Translation2d(shooterVx, shooterVy)
                            .rotateBy(robotToTargetXY.getAngle().unaryMinus())
                            .getY();

                    double angular = vOmega;
                    double distanceToTarget = Math.max(robotToTargetXY.getNorm(), 0.1);

                    double wanted = angular + tangent / distanceToTarget;

                    if (Math.hypot(vx, vy) < 0.05 && Math.abs(vOmega) < 0.02) {
                        wanted = 0.0;
                    }

                    Logger.recordOutput("Turret/Auto Aiming Currently", true);
                    Logger.recordOutput("Turret/wanted", wanted);
                    Logger.recordOutput("Turret/ff/tangent", tangent);
                    Logger.recordOutput("Turret/ff/distance", distanceToTarget);
                    Logger.recordOutput("Turret/ff/robotVx", vx);
                    Logger.recordOutput("Turret/ff/robotVy", vy);
                    Logger.recordOutput("Turret/ff/robotOmega", vOmega);
                    return wanted;
                },
                () -> false);
    }

    public Command snapThenTrack() {
        return autoAim();
    }

    public Command setPositionFieldRelative(Supplier<Double> positionRadians, Supplier<Double> velocityRadiansPerSecond,
            Supplier<Boolean> fast) {
        return setPositionSafe(
                () -> -(positionRadians.get() - RobotState.getInstance().getVisionPose().getRotation().getRadians()),
                velocityRadiansPerSecond, fast);
    }

    // public Command setPositionSafe(Supplier<Double> positionRadians, Supplier<Double> velocityRadiansPerSecond,
    //         Supplier<Boolean> fast) {
    //     return this.run(() -> {
    //         double minAngle = SubsystemConstants.turretMinAngleRadians;
    //         double maxAngle = SubsystemConstants.turretMaxAngleRadians;

    //         double wantedRadians = positionRadians.get();
    //         double[] candidates = { wantedRadians, wantedRadians + 2 * Math.PI, wantedRadians - 2 * Math.PI };

    //         double bestAngle = wantedRadians;
    //         double minDistance = 9999999.0;

    //         for (double candidate : candidates) {
    //             if (candidate >= minAngle && candidate <= maxAngle) {
    //                 double distance = Math.abs(candidate - inputs.positionRadians);
    //                 if (distance < minDistance) {
    //                     minDistance = distance;
    //                     bestAngle = candidate;
    //                 }
    //             }
    //         }

    //         if (bestAngle < minAngle) {
    //             bestAngle = minAngle;
    //         } else if (bestAngle > maxAngle) {
    //             bestAngle = maxAngle;
    //         }

    //         double velocityFF = velocityRadiansPerSecond.get() * SubsystemConstants.turretRotationsPerRadian;

    //         if (Math.abs(bestAngle - inputs.positionRadians) > Math.toRadians(180.0)) {
    //             velocityFF = 0;
    //         }

    //         setpoint = io.toRotations(bestAngle);

    //         Logger.recordOutput("Turret/wantedRadians", wantedRadians);
    //         Logger.recordOutput("Turret/clampedRadians", bestAngle);
    //         Logger.recordOutput("Turret/velocityFFRotPerSec", velocityFF);

    //         if (!fast.get()) {
    //             io.setPosition(setpoint, velocityFF);
    //         } else {
    //             io.setPositionFast(setpoint);
    //         }
    //     });
    // }

        public Command setPositionSafe(Supplier<Double> positionRadians, Supplier<Double> velocityRadiansPerSecond,
            Supplier<Boolean> fast) {
        return this.run(() -> {
            double minAngle = SubsystemConstants.turretMinAngleRadians;
            double maxAngle = SubsystemConstants.turretMaxAngleRadians;

            double wantedRadians = positionRadians.get();
            double[] candidates = { wantedRadians, wantedRadians + 2 * Math.PI, wantedRadians - 2 * Math.PI };

            double bestAngle = wantedRadians;
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

            double forwardRef = -SubsystemConstants.turretZeroOffsetRadians;
            bestAngle = forwardRef + (bestAngle - forwardRef) * 1.00;

            if (bestAngle < minAngle) {
                bestAngle = minAngle;
            } else if (bestAngle > maxAngle) {
                bestAngle = maxAngle;
            }

            double velocityFF = velocityRadiansPerSecond.get() * SubsystemConstants.turretRotationsPerRadian * 1.00;

            if (Math.abs(bestAngle - inputs.positionRadians) > Math.toRadians(180.0)) {
                velocityFF = 0;
            }

            setpoint = io.toRotations(bestAngle);

            Logger.recordOutput("Turret/wantedRadians", wantedRadians);
            Logger.recordOutput("Turret/clampedRadians", bestAngle);
            Logger.recordOutput("Turret/velocityFFRotPerSec", velocityFF);

            if (!fast.get()) {
                io.setPosition(setpoint, velocityFF);
            } else {
                io.setPositionFast(setpoint);
            }
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            this.voltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command rezeroCRT() {
        return(new InstantCommand());
        //this.runOnce(() -> io.rezeroCRT());
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
        Logger.recordOutput("Turret/readyToShoot", Math.abs(inputs.positionRadians - io.toRadians(setpoint)) < SubsystemConstants.turretShotReadyThreshold
                && !(equals(setpoint, io.toRotations(SubsystemConstants.turretMinAngleRadians))
                        || equals(setpoint, io.toRotations(SubsystemConstants.turretMaxAngleRadians))));
        return Math.abs(inputs.positionRadians - io.toRadians(setpoint)) < SubsystemConstants.turretShotReadyThreshold
                && !(equals(setpoint, io.toRotations(SubsystemConstants.turretMinAngleRadians))
                        || equals(setpoint, io.toRotations(SubsystemConstants.turretMaxAngleRadians)));
    }

    public boolean getShotReadyLenient() {
        return Math.abs(inputs.positionRadians - io.toRadians(setpoint)) < 0.35
                && !(equals(setpoint, io.toRotations(SubsystemConstants.turretMinAngleRadians))
                || equals(setpoint, io.toRotations(SubsystemConstants.turretMaxAngleRadians)));
    }

    public double getCurrent() {
        return inputs.totalCurrent;
    }
}