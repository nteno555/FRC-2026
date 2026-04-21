package frc2025.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
    private double voltage;
    private double position;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.voltage = voltage;
        inputs.position = position;
    }

    @Override
    public void setSpeed(double speed) {
        voltage = speed;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void setDeployVoltage(double voltage) {}

    @Override
    public void setDeployPosition(double position, boolean fast) {
        this.position = position;
    }

    @Override
    public void rezero(double position) {
    }

}