package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        double upperVoltage;
        double lowerVoltage;
        double upperVelocityRPM;
        double percentOutput;
    }
    
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
    public void updateInputs(ShooterIOInputs inputs);

    public void setPercentOutput(double percentOutput);
}
