/* 
 * Aaron's Note
 * 
 * The new ubot might be linear indexer like "Jack in the bot" does. 
 * It abandon the original spinning indexer, and no tower.
 * 
 * The new indexer might be one motor.
 * (I don't know if the build team would like to do two motors)
 * 
 * Use the voltage to control the speed of the indexer and 
 * compatitive to the planning bot as much as possible. 
 * 
 * Todo:
 * 1. View the code and delete/add new functions/variable to it (at least we don't need tower)
 * 2. Try to make a double motors system to spare.
 * 
 */

package frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;


public class IndexerSubsystem extends SubsystemBase {
    IndexerIO io;
    IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Indexer", inputs);
        Logger.recordOutput("Superstructure/Indexing", inputs.indexerVoltage > 1.0);
        io.updateInputs(inputs);
    }

    public Command setSpeed(Supplier<Double> indexerSpeed) {
        return this.run(() -> {
            io.setSpeed(indexerSpeed.get());
        });
    }

    public Command setIndexerVoltageWithTowerSpeed(Supplier<Double> indexerVoltage, Supplier<Double> towerSpeed) {
        return this.runEnd(() -> {
            io.setVoltage(indexerVoltage.get());
            io.setTowerSpeed(towerSpeed.get());
        }, () -> {
            io.setVoltage(0.0);
            io.setTowerSpeed(0.0);
        });
    }

    public double getCurrent() {
        return inputs.totalCurrent;
    }

//    public Command setIndexerVoltageWithTowerSpeed(Supplier<Double> indexerVoltage, Supplier<Double> towerSpeed) {
//        return this.runEnd(() -> {
//            io.setVoltage(indexerVoltage.get());
//            io.setTowerSpeed(towerSpeed.get());
//        }, () -> {
//            io.setVoltage(0.0);
//            io.setTowerSpeed(0.0);
//        });
//    }
}