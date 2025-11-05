package frc2025.subsystems.funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
    @AutoLog
    public static class FunnelIOInputs {
        //Does this need anything?
        double voltage;
        double current;
    }

    public void updateInputs(FunnelIOInputs inputs); // is this needed?
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
}
