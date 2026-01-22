package frc2025.subsystems.indexer;

public class IndexerIOSim implements IndexerIO{
    private double voltage;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerCurrent = 0;
        inputs.towerCurrent = 0;

        if(voltage > 0) {
            inputs.indexerCurrent = 9999;
            inputs.towerCurrent = 9999;
        }
        inputs.indexerVoltage = voltage;
        inputs.towerVoltage = voltage;
    }

    @Override
    public void setSpeed(double indexerSpeed, double towerSpeed) {
        voltage = indexerSpeed;
    }

    @Override
    public void setVoltage(double indexerVoltage, double towerVoltage) {
        this.voltage = indexerVoltage;
    }
}
