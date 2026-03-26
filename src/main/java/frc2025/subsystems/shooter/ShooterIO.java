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
        double towerVoltage;
        double towerRPS;
    }
    
    public void setSpeed(double speed, double acceleration);
    public void setVoltage(double voltage);
    public void updateInputs(ShooterIOInputs inputs);
}
