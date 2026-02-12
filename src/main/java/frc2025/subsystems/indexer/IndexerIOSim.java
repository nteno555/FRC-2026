package frc2025.subsystems.indexer;

import frc2025.subsystems.indexer.IndexerIOIF.IndexerIOInputs;

public class IndexerIOSim implements IndexerIOIF{
    private double voltage;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerCurrent = 0;

        if(voltage > 0) {
            inputs.indexerCurrent = 10;
        }
        inputs.indexerVoltage = voltage;
    }

    @Override
    public void setSpeed(double indexerSpeed) {
        voltage = indexerSpeed;
    }

    @Override
    public void setVoltage(double indexerVoltage) {
        this.voltage = indexerVoltage;
    }
}