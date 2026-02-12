package frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        double voltage;
        double current;
        double deplyVoltage;
        double deplyCurrent;
        double position;
    }

    void updateInputs(IntakeIOInputs inputs);
    void setPosition(double position);
    void setSpeed(double speed);
    void setVoltage(double voltage);
    void setDeplyVoltage(double deplyVoltage);
    void setDeplyPosition(double deplyPosition);
    void rezero(double position);
}

