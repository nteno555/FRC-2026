package frc2025.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class IndexerIOInputs {
        double value;
    }

    public void setValue(double value);
}
