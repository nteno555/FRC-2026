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
 * (done) 1. View the code and delete/add new functions/variable to it
 * (done) 2. Try to make a double motors system to spare.
 * 
 */

package frc2025.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import frc2025.config.SubsystemConstants;


public class IndexerSubsystem extends SubsystemBase {
    IndexerIO io;
    IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    double targetVoltage = 0.0;

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Indexer", inputs);
        Logger.recordOutput("Superstructure/Indexing", inputs.indexerVoltage > 1.0);
        Logger.recordOutput("RealOutputs/Indexer/targetVoltage", targetVoltage);
        io.updateInputs(inputs);
    }

    public Command setSpeed(Supplier<Double> indexerSpeed) {
        return this.run(() -> {
            io.setSpeed(indexerSpeed.get());
        });
    }

    public Command setVoltage(Supplier<Double> indexerVoltage) {
        targetVoltage = (indexerVoltage.get());
        return this.run(() -> {
            io.setVoltage(indexerVoltage.get());
        });
    }




    public double getCurrent() {
        return inputs.totalCurrent;
    }
}