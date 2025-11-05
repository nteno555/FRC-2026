package frc2025.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ClimberSubsystem extends SubsystemBase {
    ClimberIO io;
    ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    boolean usingVoltage = false;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/climber", inputs);
        io.updateInputs(inputs);
    }

    // Sets speed as a decimal from 0-1
    public Command setSpeed(Supplier<Double> speed) {
        return this.run(() -> {
            usingVoltage = false;
            io.setSpeed(speed.get());
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.runOnce(() -> {
            usingVoltage = true;
            io.setVoltage(voltage.get());
        });
    }
}