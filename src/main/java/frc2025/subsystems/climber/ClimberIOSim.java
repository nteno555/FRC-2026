package frc2025.subsystems.climber;

public class ClimberIOSim implements ClimberIO{
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.position = Math.random();
        inputs.rotations = Math.random();
    }

    @Override
    public void setPosition(double position) {

    }

    @Override
    public void setVoltage(double voltage) {

    }

    @Override
    public void rezero() {

    }

    @Override
    public double toRotations(double position) {
        return 0;
    }

    @Override
    public double toPositionMeters(double rotations) {
        return 0;
    }
}
