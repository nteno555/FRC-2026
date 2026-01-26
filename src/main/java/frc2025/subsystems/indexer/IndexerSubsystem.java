package frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;


public class IndexerSubsystem extends SubsystemBase {
    IndexerIO io;
    IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    boolean usingVoltage = false;
    private Trigger hasPieceTrigger = (new Trigger(() -> {return inputs.indexerCurrent > 28;})).debounce(0.1);

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Indexer", inputs);
        io.updateInputs(inputs);
    }

    // Sets speed as a decimal from 0-1
    public Command setSpeed(Supplier<Double> indexerSpeed, Supplier<Double> towerSpeed) {
        return this.run(() -> {
            usingVoltage = false;
            io.setSpeed(indexerSpeed.get(), towerSpeed.get());
        });
    }

    public Command setVoltage(Supplier<Double> indexerVoltage, Supplier<Double> towerVoltage) {
        return this.runEnd(() -> {
            usingVoltage = true;
            io.setVoltage(indexerVoltage.get(), towerVoltage.get());
        }, () -> {
            io.setVoltage(0.0, 0.0);
        });
    }

    public Command setIntake(Supplier<Boolean> intake) {
        return this.run(() -> {
            usingVoltage = false;
            if (intake.get()) io.setSpeed(1, 1);
            else io.setSpeed(0, 0);
        });
    }

    public Command setOuttake(Supplier<Boolean> intake) {
        return this.run(() -> {
            usingVoltage = false;
            if (intake.get()) io.setSpeed(-1.0, -1.0);
            else io.setSpeed(0, 0);
        });
    }

    public Command setAuto() {
        return this.run(() -> {
            if(getHasPiece()) io.setSpeed(1.0, 1.0);
            else io.setSpeed(0, 0);
        });
    }

    public IndexerIO getIO() {
        return io;
    }

    public boolean getHasPiece() {
        return hasPieceTrigger.getAsBoolean();
    }
}

