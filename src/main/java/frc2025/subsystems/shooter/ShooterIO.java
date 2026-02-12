package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        double voltage;
        double current;
        double position;
    }

    void updateInputs(ShooterIOInputs inputs);
    void setVoltage(double voltage);
    void setCurrent(double current);
    void setPosition(double position);
    void rezero(double position);
}
