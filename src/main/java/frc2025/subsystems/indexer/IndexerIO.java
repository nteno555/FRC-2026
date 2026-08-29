package frc2025.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean connected;
        double indexerVoltage;
        double indexerCurrent;
        double indexer2Voltage;
        double indexer2Current;
        double totalCurrent;
    }

    public void updateInputs(IndexerIOInputs inputs);
    public void setSpeed(double indexerSpeed);
    public void setVoltage(double indexerVoltage);
}