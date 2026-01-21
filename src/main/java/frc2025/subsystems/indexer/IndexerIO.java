package frc2025.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        double indexerVoltage;
        double indexerCurrent;
        double towerVoltage;
        double towerCurrent;
    }

    public void updateInputs(IndexerIOInputs inputs); // is this needed?
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
}
