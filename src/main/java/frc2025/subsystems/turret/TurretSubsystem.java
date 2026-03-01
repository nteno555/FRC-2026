package frc2025.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        RobotState.getInstance().setTurretVelocity(inputs.velocity);
        Logger.recordOutput("Turret/velocity", inputs.velocity);

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
                double mt1RobotRelativeAngle = -mt1FieldRelativeAngle + RobotState.getInstance().getVisionPose().getRotation().getRadians();
                io.rezero(io.toRotations(mt1RobotRelativeAngle));
            }
            RobotState.getInstance().setTurretMt1Valid(false);
        }
    }

    public Command autoAim() {
        return setPositionFieldRelative(
                () -> {
                    double wantedAngle = RobotState.getInstance().getAngleToVirtualTarget();
                    double visionAngle = RobotState.getInstance().getVisionPose().getRotation().getRadians();
                    Logger.recordOutput("Turret/error", Math.abs(inputs.positionRadians + visionAngle - wantedAngle) % (2*Math.PI));
                    return wantedAngle;
                },
                () -> {
                    ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
                    Translation2d targetPos = RobotState.getInstance().getShootingWhileMovingVirtualPose().getTranslation();
                    var robotPose = RobotState.getInstance().getVisionPose();
                    Translation2d robotPos = robotPose.getTranslation();

                    var robotToTargetXY = targetPos.minus(robotPos);
                    Translation2d shooterRobotRelative = SubsystemConstants.robotToShooter.getTranslation().rotateBy(robotPose.getRotation());

                    robotSpeeds = new ChassisSpeeds(
                        robotSpeeds.vxMetersPerSecond + robotSpeeds.omegaRadiansPerSecond * shooterRobotRelative.getY(),
                        robotSpeeds.vyMetersPerSecond - robotSpeeds.omegaRadiansPerSecond * shooterRobotRelative.getX(),
                            robotSpeeds.omegaRadiansPerSecond
                    );

                    var targetFrameToRobot = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                            .rotateBy(
                                    robotToTargetXY.getAngle());

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
            io.setPosition(setpoint, velocityFF);
            Logger.recordOutput("Turret/turretWantedPositionRadians", positionRadians.get());
            Logger.recordOutput("Turret/turretClampedPositionRadians", bestAngle);
            Logger.recordOutput("Turret/turretClampedPositionRotations", setpoint);
        });
    }

    public Command setPosition(Supplier<Double> position) {
        return this.run(() -> {
            setpoint = position.get();
            usingVoltage = false;
            io.setPosition(setpoint);
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

    public Command setFixed() {
        return setPosition(() -> {
            if(usingVoltage) setpoint = inputs.positionTicks;
            return setpoint;
        });
    }

    public double getPosition() { return inputs.positionTicks; }
    public double getPositionRadians() { return inputs.positionRadians; }

    public boolean getShotReady() {
        return Math.abs(inputs.positionRadians - io.toRadians(setpoint)) < SubsystemConstants.turretShotReadyThreshold;
    }

}