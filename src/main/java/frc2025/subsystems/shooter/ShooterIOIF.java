package frc2025.subsystems.shooter;

public interface ShooterIOIF {
    double voltage = 0;
    double current = 0;
    double position = 0;

    void setVoltage(double voltage);
    void setCurrent(double current);
    void setPosition(double position);

    double getVoltage();
    double getCurrent();
    double getPosition();
}
