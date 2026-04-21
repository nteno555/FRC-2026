package frc2025.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public boolean connected;
        double climbVoltage;
        double climbCurrent;
        double towerVoltage;
        double towerCurrent;
        double totalCurrent;
    }

    public void updateInputs(ClimbIOInputs inputs);
    public void setSpeed(double climbSpeed);
    public void setVoltage(double climbVoltage);
}