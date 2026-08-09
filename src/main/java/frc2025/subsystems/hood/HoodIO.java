package frc2025.subsystems.hood;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO{
    @AutoLog
    public class HoodIOInputs{
        public double positionRotations, positionRadians, positionDegreesReal, totalCurrent;
    }
    public void updateInputs(HoodIOInputs inputs);
    public void setPosition(double position);
    public void setPosition(double position, double velocity);
    public void setVoltage(double voltage);
    public void rezero(double position);
}