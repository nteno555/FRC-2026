package frc2025.subsystems.intake;

public interface IntakeIOIF {
    double towerVoltage = 0;
    double towerCurrent = 0;
    double rollerVoltage = 0;
    double rollerCurrent = 0;
    double position = 0;
    
    void setTowerVoltage(double voltage);
    void setTowerCurrent(double current);
    void setRollerVoltage(double voltage);
    void setRollerCurrent(double current);
    void setPosition(double position);

    double getTowerVoltage();
    double getTowerCurrent();
    double getRollerVoltage();
    double getRollerCurrent();
    double getPosition();

    void periodic();
}

