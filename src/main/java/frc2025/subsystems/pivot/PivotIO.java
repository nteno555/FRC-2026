package frc2025.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    public class PivotIOInputs {
        public double positionRotations;
        public double positionDegrees;
        public double positionDegreesReal;
    }

    public void updateInputs(PivotIOInputs inputs);
    public void setPosition(double position);
    public void setVoltage(double voltage);
    public void rezero(double position);

}
