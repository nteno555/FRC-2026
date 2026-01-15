package frc2025.subsystems.turret;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        double positionTicks;
        double positionDegrees;
        double positionDegreesReal;
    }

    public void updateInputs(TurretIOInputs inputs);
    public void setPosition(double position);
    public void setVoltage(double voltage);
    public void rezero(double position);
}
