package frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.Robot;
import frc2025.RobotState;
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

    public Command autoShoot() {
        return this.setVoltage(() -> {
            if (RobotState.getInstance().getDriveShotReady() && RobotState.getInstance().getDriveShotReady() && RobotState.getInstance().getWantAutoShoot()) return 9.0;
            return 0.0;
        }, () -> {
            if (RobotState.getInstance().getDriveShotReady() && RobotState.getInstance().getDriveShotReady() && RobotState.getInstance().getWantAutoShoot()) return 12.0;
            return 0.0;
        });
    }
}

