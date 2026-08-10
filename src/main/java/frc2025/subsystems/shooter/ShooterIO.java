package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        double alignedVoltage;
        double alignedVelocityRPS;

        double opposedAVoltage;
        double opposedAVelocityRPS;

        double opposedBVoltage;
        double opposedBVelocityRPS;

        double masterVoltage;
        double masterVelocityRPS;

        double percentOutput;
        double towerVoltage;
        double towerRPS;
        double totalCurrent;
    }
    
    public void setSpeed(double speed, double acceleration);
    public void setVoltage(double voltage);
    public void setCurrent(double currentAmps);
    public void updateInputs(ShooterIOInputs inputs);
}
