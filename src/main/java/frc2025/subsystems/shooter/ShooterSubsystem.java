package frc2025.subsystems.shooter;

import java.util.function.Supplier;

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
        return this.run(() -> {
            Logger.recordOutput("Shooter/wantedSpeedSubsystem", speed.get());
            usingVoltage = false;
            wantedVelocity = speed.get();
            io.setSpeed(speed.get());
        });
    }

    public double getUpperVelocity() {
        return inputs.upperVelocityRPM;
    }
    public double getUpperWantedVelocity() {
        return wantedVelocity;
    }
    public boolean getShotReady() {
        return Math.abs(wantedVelocity - inputs.upperVelocityRPM) < 100.0;
    }
}