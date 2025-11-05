package frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        //Does this need anything?
        double voltage;
    }

    public void updateInputs(ClimberIOInputs inputs); // is this needed?
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
}
