package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        double slaveA1Voltage, slaveA1VelocityRPS;
        double slaveB1Voltage, slaveB1VelocityRPS;
        double slaveB2Voltage, slaveB2VelocityRPS;
        double masterVoltage, masterVelocityRPS;
        double percentOutput;
        double masterStatorCurrent,slaveA1StatorCurrent,slaveB1StatorCurrent,slaveB2StatorCurrent;
        double totalCurrent;
    }

    // public default void setSpeed(double speed){}
    public void setSpeed(double speed, double acceleration);
    public void setVoltage(double voltage);
    public void updateInputs(ShooterIOInputs inputs);
    public default void setPercentOutput(double percentOutput){}
}