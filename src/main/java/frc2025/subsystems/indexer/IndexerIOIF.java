package frc2025.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIOIF {
    @AutoLog
    public static class IndexerIOInputs{
    double indexerVoltage; 
    double indexerCurrent;}

    public void updateInputs(IndexerIOInputs inputs);
    public void setSpeed(double indexerSpeed);
    public void setVoltage(double indexerVoltage);
    
} 
