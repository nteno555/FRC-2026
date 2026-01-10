package frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs{
        public double voltage;
        public double current;
        public double position;
    }
    public void updateInputs(IntakeIOInputs inputs);
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
    public void setPosition(double position, boolean useVoltage);
}