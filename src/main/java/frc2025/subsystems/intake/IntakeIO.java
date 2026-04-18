package frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        //lin prefix are attributes of the linear motor and rot prefix signifies values for the intake motor ON THe front
        double linVoltage, linCurrent, position, rotVoltage, rotCurrent, logLinVoltage, logLinCurrent, logPosition, logRotVoltage, logRotCurrent;
        
    }
    public void updateInputs(IntakeIOInputs inputs);
    public void setPosition(double position); //horizontal position of the intake
    public void setLinVoltage(double linVoltage);
    public void setRotVoltage(double rotVoltage);

}
