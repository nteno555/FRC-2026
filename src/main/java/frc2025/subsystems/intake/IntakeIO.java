package frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        double canRangeDistance;
        double voltage;
        double current;
    }
    public void updateInputs(IntakeIOInputs inputs);
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
}
