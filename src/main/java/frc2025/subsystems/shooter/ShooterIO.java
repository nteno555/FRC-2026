package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        double slaveVoltage;
        double slaveVelocityRPM;
        double masterVoltage;
        double masterVelocityRPM;
        double percentOutput;
    }
    
    public void setSpeed(double speed);
    public void setVoltage(double voltage);
    public void updateInputs(ShooterIOInputs inputs);
    public void setPercentOutput(double percentOutput);
//    public boolean getShotReady(double wantedSpeed);
}
