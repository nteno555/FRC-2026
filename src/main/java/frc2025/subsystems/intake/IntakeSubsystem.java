package frc2025.subsystems.intake;

import frc2025.subsystems.shooter.ShooterIOIF;

public class IntakeSubsystem {
    IntakeIOIF io;

    public IntakeSubsystem(IntakeIOIF io) {
        this.io = io;
    }   

    public void rezero() {
        setPosition(0);
        io.setRollerVoltage(0);
        io.setTowerVoltage(0);
    }

    public void setRollerVoltage(double voltage) {io.setRollerVoltage(voltage);}
    public void setDeployVoltage(double voltage) {io.setTowerVoltage(voltage);}
    public void setPosition(double position) {io.setPosition(position);}
    

    public double getRollerVoltage() {return io.getRollerVoltage();}
    public double getDeployVoltage() {return io.getTowerVoltage();}
    public double getPosition() {return io.getPosition();}
}
