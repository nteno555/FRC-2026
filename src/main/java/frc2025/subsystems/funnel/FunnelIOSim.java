package frc2025.subsystems.funnel;


import org.littletonrobotics.junction.Logger;

public class FunnelIOSim implements FunnelIO {
    double voltage = 0;
    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        inputs.voltage = voltage;
    }

    @Override
    public void setSpeed(double speed) {
        voltage = speed;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
