package frc2025.subsystems.intake;

public interface IntakeIOIF {
    double towerVoltage = 0;
    double towerCurrent = 0;
    double towerPosition = 0;
    double rollerVoltage = 0;
    double rollerCurrent = 0;
    double rollerPosition = 0;
    
    void setTowerVoltage(double voltage);
    void setTowerCurrent(double current);
    void setRollerVoltage(double voltage);
    void setRollerCurrent(double current);
    void setTowerPosition(double position);
    void setRollerPosition(double position);

    double getTowerVoltage();
    double getTowerCurrent();
    double getRollerVoltage();
    double getRollerCurrent();
    double getTowerPosition();
    double getRollerPosition();
}

