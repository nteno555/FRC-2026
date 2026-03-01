package frc2025.subsystems.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.RobotState;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.function.Supplier;

public class HoodSubsystem extends SubsystemBase {
    HoodIO io;
    HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    double setpoint;
    double voltage;
    boolean usingVoltage = false;
    double voltageRamp = 0;

    public HoodSubsystem(HoodIO io) {
        io.rezero(0);
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Hood", inputs);
        Logger.recordOutput("Hood/setpoint", setpoint);
        if(DriverStation.isDisabled()) {
            setpoint = inputs.positionRotations;
        }

        io.updateInputs(inputs);
    }

    public Command setPositionRadians(Supplier<Double> positionRadians){
        return setPosition(() -> toRotations(positionRadians.get()));
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

    public Command autoAim() {
        return setPosition(() -> {
            if (usingVoltage) {
                return inputs.positionRotations;
            }

            ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldRelativeSpeeds();
            double dt = 1.0;
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            double robotXpose = robotPose.getX();
            double robotYpose = robotPose.getY();

            Pose2d futureRobotPose = new Pose2d(
                    robotXpose + robotVelocity.vxMetersPerSecond * dt,
                    robotYpose + robotVelocity.vyMetersPerSecond * dt,
                    Rotation2d.kZero
            );

            boolean nearBlueTrench = Math.abs(robotXpose - FieldConstants.blueTrenchX) < FieldConstants.trenchThreshold;
            boolean nearRedTrench = Math.abs(robotXpose - FieldConstants.redTrenchX) < FieldConstants.trenchThreshold;
            boolean nearTrenchY = robotYpose < 1.982 || robotYpose > 6.170;

            boolean currentBlueTrench = (robotXpose - FieldConstants.blueTrenchX) < 0.0;
            boolean futureBlueTrench = (futureRobotPose.getX() - FieldConstants.blueTrenchX) < 0.0;
            boolean currentRedTrench = (robotXpose - FieldConstants.redTrenchX) < 0.0;
            boolean futureRedTrench = (futureRobotPose.getX() - FieldConstants.redTrenchX) < 0.0;

            Logger.recordOutput("Hood/nearBlueTrench", nearBlueTrench);
            Logger.recordOutput("Hood/nearRedTrench", nearRedTrench);

            if ((nearRedTrench || nearBlueTrench) && nearTrenchY) {
                return 0.0;
            }

            if(DriverStation.isAutonomous() && (currentBlueTrench != futureBlueTrench || currentRedTrench != futureRedTrench)) {
                return 0.0;
            }

            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());

            if (RobotState.getInstance().getShuttling()) {
                return SubsystemConstants.hoodMaxAngleRotations;
//                return ShooterTreeMap.getInstance().shuttlingHoodTreeMap.getInterpolated(dist);
            }

            return ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(dist);
        });
    }

    public Command autoAimFlush() {
        return setPosition(() -> ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(1.0));
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

    public Command homeThenRezero() {
        return this.run(() -> {
            usingVoltage = true;
            io.setVoltage(SubsystemConstants.hoodRezeroVoltage);
            io.rezero(0.0);
        }).andThen(() -> {
            usingVoltage = false;
            io.rezero(0.0);
        });
    }

    public Command setFixed() {
        return setPosition(() -> {
            if(usingVoltage) setpoint = inputs.positionRotations;
            return setpoint;
        });
    }

    public boolean isNear(double position) {
        return Math.abs(inputs.positionRotations - position) < 2.5;
    }
    public double getPosition() { return inputs.positionRotations; }
    public double getPositionRadians() { return inputs.positionRadians; }

    public static double toRadians(double rotations) {
        double m = (SubsystemConstants.hoodMaxAngleRadians - SubsystemConstants.hoodMinAngleRadians) / (SubsystemConstants.hoodMaxAngleRotations - SubsystemConstants.hoodMinAngleRotations);
        return (rotations - SubsystemConstants.hoodMinAngleRotations) * m + SubsystemConstants.hoodMinAngleRadians;
    }
    public static double toRotations(double radians) {
        double m = (SubsystemConstants.hoodMaxAngleRotations - SubsystemConstants.hoodMinAngleRotations) / (SubsystemConstants.hoodMaxAngleRadians - SubsystemConstants.hoodMinAngleRadians);
        return (radians - SubsystemConstants.hoodMinAngleRadians) * m + SubsystemConstants.hoodMinAngleRotations;
    }
}
