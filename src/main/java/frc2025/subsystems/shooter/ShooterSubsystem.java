package frc2025.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
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
            io.setVoltage(voltage.get());
        });
    }

    public Command setSpeed(Supplier<Double> speed) {
        return this.run(() -> {
            io.setSpeed(speed.get());
        });
    }
}