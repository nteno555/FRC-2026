package frc2025.subsystems.turret;

import edu.wpi.first.wpilibj.DriverStation;

public class TurretIOSim implements TurretIO{
    private final double ROTATIONS_PER_DEGREE = 0.20833333333;
    private double targetPosition = 0;
    private double velocity = 0;
    private boolean useVoltage = false;
    private boolean init = false;

    private double toDegree(double rotations) {
        return 1/ROTATIONS_PER_DEGREE * rotations;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        if(!init) inputs.positionTicks = 0;
        init = true;

        if(DriverStation.isEnabled()) {
            if (!useVoltage) velocity = 0.1 * (targetPosition - inputs.positionTicks);
        } else {
            velocity = 0;
        }

        inputs.positionTicks = inputs.positionTicks + velocity;
        inputs.positionDegrees = toDegree(inputs.positionTicks);
        inputs.positionDegreesReal = inputs.positionDegrees + 15;
    }

    @Override
    public void setPosition(double position) {
        useVoltage = false;
        targetPosition = position;
    }

    @Override
    public void setPosition(double position, double velocity) {

    }

    @Override
    public void setVoltage(double voltage) {
        useVoltage = true;
        velocity = voltage;
    }

    @Override
    public void rezero() {}

    @Override
    public void rezero(double position) {

    }

    @Override
    public void rezeroCRT() {

    }

    @Override
    public double toRadians(double position) {
        return 0;
    }

    @Override
    public double toRotations(double radians) {
        return 0;
    }
}