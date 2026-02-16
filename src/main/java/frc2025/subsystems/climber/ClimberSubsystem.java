package frc2025.subsystems.climber;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ClimberSubsystem extends SubsystemBase {
    ClimberIO io;
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public ClimberSubsystem(ClimberIO io) {
        io.rezero();
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Climber", inputs);
        io.updateInputs(inputs);
    }

    public Command setPosition(Supplier<Double> position) {
        return this.run(() -> {
            io.setPosition(position.get());
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            io.setVoltage(voltage.get());
        });
    }

    public Command rezero() {
        return this.runOnce(() -> {
            io.rezero();
        });
    }
}
