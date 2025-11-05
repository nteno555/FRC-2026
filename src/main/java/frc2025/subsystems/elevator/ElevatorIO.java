package frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public double positionRotations = 0.0;
        public double voltage = 0.0;
    }

    public void updateInputs(ElevatorIOInputs inputs);
    public void setPosition(double meters);
    public void setPositionSlow(double meters);
    public void setVoltage(double voltage);
    public void setCurrent(double amps);
    public void rezero(double position);
    public void applyFeedForwardConstants(boolean hasPiece);

}
