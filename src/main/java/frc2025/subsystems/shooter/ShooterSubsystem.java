package frc2025.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import frc2025.RobotState;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    double wantedVelocity = 0.0;
    double wantedVoltage = 0.0;
    boolean usingVoltage = false;
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Shooter", inputs);
        Logger.recordOutput("Superstructure/ShooterSpeed", inputs.masterVelocityRPM);
        io.updateInputs(this.inputs);
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            Logger.recordOutput("Shooter/wantedVoltage", voltage.get());
            usingVoltage = true;
            wantedVoltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command setSpeed(Supplier<Double> speed) {
        return this.runEnd(() -> {
            Logger.recordOutput("Shooter/wantedSpeed", speed.get());
            usingVoltage = false;
            wantedVelocity = speed.get();
            io.setSpeed(speed.get(), 0.0);
        }, () -> {
            usingVoltage = false;
            io.setSpeed(0.0, 0.0);
        });
    }

    public Command setSpeed(Supplier<Double> speed, Supplier<Double> acceleration) {
        return this.runEnd(() -> {
            Logger.recordOutput("Shooter/wantedSpeed", speed.get());
            usingVoltage = false;
            wantedVelocity = speed.get();
            io.setSpeed(speed.get(), acceleration.get());
        }, () -> {
            usingVoltage = false;
            io.setSpeed(0.0, 0.0);
        });
    }

    public Command autoAim() {
        return setSpeed(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());

            if (RobotState.getInstance().getShuttling()) {
                return ShooterTreeMap.getInstance().shuttlingRPSTreeMap.getInterpolated(dist);
            }

            return ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
        }, () -> {
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());

            ChassisSpeeds robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
            Translation2d robotToTargetXY = targetPose.getTranslation().minus(robotPose.transformBy(SubsystemConstants.robotToShooter).getTranslation());
            double distanceToTarget = robotToTargetXY.getNorm();

            double radialVelocity = (robotSpeeds.vxMetersPerSecond * robotToTargetXY.getX() + robotSpeeds.vyMetersPerSecond * robotToTargetXY.getY()) / distanceToTarget;
            
            double dRps_dDist;
            if (RobotState.getInstance().getShuttling()) {
                dRps_dDist = (ShooterTreeMap.getInstance().shuttlingRPSTreeMap.getInterpolated(dist + 0.05) - ShooterTreeMap.getInstance().shuttlingRPSTreeMap.getInterpolated(dist - 0.05)) / 0.1;
            } else {
                dRps_dDist = (ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist + 0.05) - ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist - 0.05)) / 0.1;
            }
            double acceleration = dRps_dDist * (-radialVelocity);
            return acceleration;
        });
    }

    public boolean getShotReady() {
        return Math.abs(wantedVelocity - inputs.masterVelocityRPM) < 10.0 && inputs.masterVelocityRPM > 1.0;
    }

    public double getVelocity() {
        return inputs.masterVelocityRPM;
    }

}