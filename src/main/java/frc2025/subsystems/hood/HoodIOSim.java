package frc2025.subsystems.hood;

import edu.wpi.first.wpilibj.DriverStation;

public class HoodIOSim implements HoodIO {
    private double targetPosition = 0;


    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRotations = targetPosition;
    }

    @Override
    public void setPosition(double position) {
        targetPosition = position;
    }

    @Override
    public void setPosition(double position, double velocity) {
        targetPosition = position;
    }

    @Override
    public void setVoltage(double voltage) {
        targetPosition += voltage * 0.1;
    }

    @Override
    public void rezero(double position) {
        targetPosition = 0;
    }
}
