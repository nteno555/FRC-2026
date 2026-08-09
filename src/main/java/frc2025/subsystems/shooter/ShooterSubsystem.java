package frc2025.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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
        Logger.recordOutput("Superstructure/ShooterSpeed", inputs.masterVelocityRPS);
        Logger.recordOutput("Shooter/wantedSpeed", wantedVelocity);
        io.updateInputs(this.inputs);
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            wantedVoltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command setSpeed(Supplier<Double> speed) {
        return this.runEnd(() -> {
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
            Pose2d robotPose = RobotState.getInstance().getDelayedShooterPose();
            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());

            if (RobotState.getInstance().getShuttling()) {
                return ShooterTreeMap.getInstance().shuttlingRPSTreeMap.getInterpolated(dist);
            }

            return ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(dist);
        }, () -> {
            // return 0.0;
            return RobotState.getInstance().getWantedShooterAcceleration();
        });
    }

    public boolean getShotReady() {
        return Math.abs(wantedVelocity - inputs.masterVelocityRPS) < 5.0 && inputs.masterVelocityRPS > 1.0;
    }

    public double getCurrent() {
        return inputs.totalCurrent;
    }

    public double getVelocity() {
        return inputs.masterVelocityRPS;
    }
}