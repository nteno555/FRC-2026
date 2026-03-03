package frc2025.subsystems.turret;

import edu.wpi.first.wpilibj.DriverStation;
import frc2025.config.SubsystemConstants;

public class TurretIOSim implements TurretIO{
    private double targetPosition = 0;

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionTicks = targetPosition;
        inputs.positionRadians = toRadians(inputs.positionTicks);
        inputs.tb1Connected = true;
        inputs.tb2Connected = true;
        inputs.throughboresConnected = true;
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
    public void setPositionFast(double position) {
        targetPosition = position;
    }

    @Override
    public void setVoltage(double voltage) {
        targetPosition += voltage * 0.1;
    }

    @Override
    public void rezero() {

    }

    @Override
    public void rezero(double position) {

    }

    @Override
    public void rezeroCRT() {

    }

    @Override
    public void addVisionUpdate(double robotRelativeAngleRadians) {

    }

    public double toRadians(double position) {
        return 1/ SubsystemConstants.turretRotationsPerRadian * position;
    }
    public double toRotations(double radians) {
        return SubsystemConstants.turretRotationsPerRadian * radians;
    }
}