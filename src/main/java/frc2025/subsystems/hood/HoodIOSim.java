package frc2025.subsystems.hood;

import edu.wpi.first.wpilibj.DriverStation;

public class HoodIOSim implements HoodIO {
    private final double ROTATIONS_PER_DEGREE = 0.20833333333;
    private double targetPosition = 0;
    private double velocity = 0;
    private boolean useVoltage = false;
    private boolean init = false;

    private double toDegree(double rotations) {
        return 1/ROTATIONS_PER_DEGREE * rotations;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        if(!init) inputs.positionRotations = -36;
        init = true;

        if(DriverStation.isEnabled()) {
            if (!useVoltage) velocity = 0.1 * (targetPosition - inputs.positionRotations);
        } else {
            velocity = 0;
        }

        inputs.positionRotations = inputs.positionRotations + velocity;
        inputs.positionRadians = toDegree(inputs.positionRotations);
        inputs.positionDegreesReal = inputs.positionRadians + 15;
    }

    @Override
    public void setPosition(double position) {
        useVoltage = false;
        targetPosition = position;
    }

    @Override
    public void setVoltage(double voltage) {
        useVoltage = true;
        velocity = voltage;
    }

    @Override
    public void rezero(double position) {
        targetPosition = 0;
    }
}
