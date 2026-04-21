package frc2025.subsystems.climb;

import frc2025.config.SubsystemConstants;

public class ClimbIOSim implements ClimbIO {
    private double voltage;

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbCurrent = 0;
        if (SubsystemConstants.hasTower) inputs.towerCurrent = 0;

        if(voltage > 0) {
            inputs.climbCurrent = 9999;
            if (SubsystemConstants.hasTower) inputs.towerCurrent = 9999;
        }
        inputs.climbVoltage = voltage;
        if (SubsystemConstants.hasTower) inputs.towerVoltage = voltage;
    }

    @Override
    public void setSpeed(double climbSpeed) {
        voltage = climbSpeed;
    }

    @Override
    public void setVoltage(double climbVoltage) {
        this.voltage = climbVoltage;
    }
}