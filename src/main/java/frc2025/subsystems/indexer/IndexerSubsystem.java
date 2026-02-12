package frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class IndexerSubsystem extends SubsystemBase {
    IndexerIOIF io;
    IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    boolean usingVoltage = false;

    public IndexerSubsystem(IndexerIOIF io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Indexer", inputs);
        io.updateInputs(inputs);
    }

    public Command setSpeed(Supplier<Double> indexerSpeed) {
        return this.run(() -> {
            usingVoltage = false;
            io.setSpeed(indexerSpeed.get());
        });
    }

    public Command setVoltage(Supplier<Double> indexerVoltage) {
        return this.runEnd(() -> {
            usingVoltage = true;
            io.setVoltage(indexerVoltage.get());
        }, () -> {
            io.setVoltage(0.0);
        });
    }
}