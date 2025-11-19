package frc2025.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {
    private double voltage;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.upperVoltage = voltage;
        inputs.lowerVoltage = voltage;
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

