package frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        double voltage;
        double current;
        double position;
        boolean deployConnected;
    }
    public void updateInputs(IntakeIOInputs inputs);
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
    public void setDeployVoltage(double voltage);
    public void setDeployPosition(double position);
    public void rezero(double position);
//    public void setZeroPosition();
}
