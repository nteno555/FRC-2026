package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        double slaveA1Voltage,slaveA1VelocityRPS;
        double slaveB1Voltage,slaveB1VelocityRPS;
        double slaveB2Voltage,slaveB2VelocityRPS;
        double masterVoltage,masterVelocityRPS;
        double percentOutput;
        double towerVoltage;
        double towerRPS;
        double totalCurrent;
    }


    public void setVoltage(double voltage);
    public void updateInputs(ShooterIOInputs inputs);
    public void setPercentOutput(double percentOutput);
    public void setSpeed(double speed);
}
