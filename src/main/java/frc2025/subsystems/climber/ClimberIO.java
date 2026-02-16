package frc2025.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        double rotations;
        double position;
    }

    public void updateInputs(ClimberIOInputs inputs);
    public void setPosition(double position);
    public void setVoltage(double voltage);
    public void rezero();

    public double toRotations(double position);
    public double toPositionMeters(double rotations);
}