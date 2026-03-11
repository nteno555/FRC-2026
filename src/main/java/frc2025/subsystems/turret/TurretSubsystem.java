package frc2025.subsystems.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.Robot;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
    TurretIO io;
    TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    double setpoint, voltage, voltageRamp;
    boolean usingVoltage = false;
    boolean zeroed = false;

    public TurretSubsystem(TurretIO io) {
        this.io = io;
        io.rezero();
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Turret", inputs);
        Logger.recordOutput("Turret/setpoint", setpoint);
        if(DriverStation.isDisabled()) {setpoint = inputs.positionTicks;}
        io.updateInputs(inputs);

        double fieldRelativeAngleRad = -inputs.positionRadians + RobotState.getInstance().getVisionPose().getRotation().getRadians();
        Logger.recordOutput("Turret/fieldRelativeAngleRadians", fieldRelativeAngleRad);
        Logger.recordOutput("Vision/aimAngleDiff", RobotState.getInstance().getAngleToVirtualTarget() - fieldRelativeAngleRad);

        RobotState.getInstance().setTurretAngle(inputs.positionRadians);

        if(SubsystemConstants.hasTurret) {
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

        if(RobotState.getInstance().getTurretMt1Valid() && SubsystemConstants.useTurretVisionCorrection) {
            double mt1FieldRelativeAngle = RobotState.getInstance().getTurretMt1AngleRadians();
            if(Math.abs(mt1FieldRelativeAngle - fieldRelativeAngleRad) > 0.1) {
                double mt1RobotRelativeAngle = (-mt1FieldRelativeAngle + RobotState.getInstance().getVisionPose().getRotation().getRadians());
                Logger.recordOutput("Turret/mt1RobotRelativeAngle", mt1RobotRelativeAngle);
                Logger.recordOutput("Turret/visionRezeroHeartbeat", Math.random());


                io.rezero(io.toRotations(inputs.positionRadians + 0.01 * (mt1RobotRelativeAngle - inputs.positionRadians)));
                //io.rezero(io.toRotations(mt1RobotRelativeAngle));
            }
            RobotState.getInstance().setTurretMt1Valid(false);
        }

        Logger.recordOutput("Superstructure/TurretAngleRadians", inputs.positionRadians);
        Logger.recordOutput("Turret/turretPose",
                new Pose2d(
                        RobotState.getInstance().getVisionShooterPose().getTranslation(),
                        Rotation2d.fromRadians(fieldRelativeAngleRad)));
    }

    public Command autoAim() {
        return setPositionFieldRelative(
                () -> {
                    double wantedAngle = RobotState.getInstance().getAngleToVirtualTarget();
                    double visionAngle = RobotState.getInstance().getVisionPose().getRotation().getRadians();
                    Logger.recordOutput("Turret/error", (Math.abs(inputs.positionRadians + visionAngle - wantedAngle) % (2*Math.PI)));
                    return wantedAngle;
                },
                () -> {
                    ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
                    double vx = robotSpeeds.vxMetersPerSecond;
                    double vy = robotSpeeds.vyMetersPerSecond;
                    double vOmega = robotSpeeds.omegaRadiansPerSecond;

                    Translation2d targetPos = RobotState.getInstance().getShootingWhileMovingVirtualPose().getTranslation();
                    var robotPose = RobotState.getInstance().getVisionPose();
                    Translation2d shooterPos = robotPose.transformBy(SubsystemConstants.robotToShooter).getTranslation();

                    var robotToTargetXY = targetPos.minus(shooterPos);
                    Translation2d shooterRobotRelative = SubsystemConstants.robotToShooter.getTranslation().rotateBy(robotPose.getRotation());

                    robotSpeeds = new ChassisSpeeds(
                        vx + vOmega * shooterRobotRelative.getY(),
                        vy - vOmega * shooterRobotRelative.getX(),
                            vOmega
                    );

                    var targetFrameToRobot = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                            .rotateBy(robotToTargetXY.getAngle());

                    var tangent = targetFrameToRobot.getY();
                    var angular = robotSpeeds.omegaRadiansPerSecond;
                    var distanceToTarget = robotToTargetXY.getNorm();

                    return (angular + tangent / distanceToTarget);
                }
        );
    }

    public Command setPositionFieldRelative(Supplier<Double> positionRadians, Supplier<Double> velocityRadiansPerSecond) {
        return setPositionSafe(() -> -(positionRadians.get() - RobotState.getInstance().getVisionPose().getRotation().getRadians()),
                velocityRadiansPerSecond);
    }

//    public Command setPositionSafe(Supplier<Double> positionRadians) {
//        return setPositionSafe(positionRadians, () -> 0.0);
//    }

    public Command setPositionSafe(Supplier<Double> positionRadians, Supplier<Double> velocityRadiansPerSecond) {
        return this.run(() -> {
            double minAngle = SubsystemConstants.turretMinAngleRadians;
            double maxAngle = SubsystemConstants.turretMaxAngleRadians;

            double[] candidates = {positionRadians.get(), positionRadians.get() + 2*Math.PI, positionRadians.get() - 2*Math.PI};

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
            }
            else if (bestAngle > maxAngle) {
                bestAngle = maxAngle;
            }

            double velocityFF = velocityRadiansPerSecond.get() * SubsystemConstants.turretRotationsPerRadian;

            if(Math.abs(bestAngle - inputs.positionRadians) > Math.toRadians(180.0)){
                velocityFF = 0;
            }


            setpoint = io.toRotations(bestAngle);

            usingVoltage = false;
            //io.setPosition(setpoint, velocityFF);


            if(io.toRadians(Math.abs(setpoint - inputs.positionTicks)) < 0.5) {
                Logger.recordOutput("Turret/slot0heartbeat", Math.random());
                io.setPosition(setpoint, velocityFF);
            } else {
                Logger.recordOutput("Turret/slot1heartbeat", Math.random());
                io.setPositionFast(setpoint);
            }
            Logger.recordOutput("Turret/turretWantedPositionRadians", positionRadians.get());
            Logger.recordOutput("Turret/turretClampedPositionRadians", bestAngle);
            Logger.recordOutput("Turret/turretClampedPositionRotations", setpoint);
        });
    }

    public Command setPosition(Supplier<Double> position) {
        return this.run(() -> {
            setpoint = position.get();
            usingVoltage = false;

        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            this.voltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command voltageRampCommand() {
        return this.runOnce(() -> { voltageRamp = 0.0; })
                .andThen(
                        this.run(() -> {
                            usingVoltage = true;
                            io.setVoltage(voltageRamp);
                            voltageRamp += 0.01;
                        }));
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


    public double getPosition() { return inputs.positionTicks; }
    public double getPositionRadians() { return inputs.positionRadians; }

    public boolean equals(double a, double b) {
        double epsilon = 0.001;
        return Math.abs(a - b) < epsilon;
    }

    public boolean getShotReady() {
        Logger.recordOutput("Turret/errorRots", Math.abs(inputs.positionTicks - setpoint));
        Logger.recordOutput("Turret/errorRads", Math.abs(inputs.positionRadians - io.toRadians(setpoint)));
        return Math.abs(inputs.positionTicks - setpoint) < io.toRotations(0.052)
                && !(equals(setpoint, io.toRotations(SubsystemConstants.turretMinAngleRadians)) || equals(setpoint, io.toRotations(SubsystemConstants.turretMaxAngleRadians)));
//        return Math.abs(inputs.positionRadians - io.toRadians(setpoint)) < SubsystemConstants.turretShotReadyThreshold;
    }

}