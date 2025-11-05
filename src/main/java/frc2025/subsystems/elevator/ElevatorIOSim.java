package frc2025.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private double velocity = 0;
    private double targetPosition = 0;
    private boolean useVoltage = false;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        velocity = 0.1 * (targetPosition - inputs.positionRotations);
        inputs.positionRotations += velocity;
    }

    @Override
    public void setPosition(double meters) {
        useVoltage = false;
        targetPosition = meters;
    }

    @Override
    public void setPositionSlow(double meters) {
        useVoltage = false;
        targetPosition = meters;
    }

    @Override
    public void setVoltage(double voltage) {
        useVoltage = true;
        velocity = voltage;
    }

    @Override
    public void setCurrent(double amps) {

    }

    @Override
    public void rezero(double position) {

    }

    @Override
    public void applyFeedForwardConstants(boolean hasPiece) {

    }
}
