package frc2025.subsystems.arm;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {
        public double positionRotations;
        public double positionDegrees;
        public double positionDegreesReal;
    }

    public void updateInputs(ArmIOInputs inputs);
    public void setPosition(double position);
    public void setVoltage(double voltage);
    public void rezero(double position);
    public void applyFeedForwardConstants(boolean hasPiece);
}
