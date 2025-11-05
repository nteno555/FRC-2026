package frc2025.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO{
    @AutoLog
    public static class EndEffectorIOInputs {
        double canRangeDistance;
        double voltage;
        double current;
    }

    public void updateInputs(EndEffectorIOInputs inputs); // is this needed?
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
}
