package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
    private double voltage;
    private double percentOutput;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.slaveVelocityRPS = voltage;
        inputs.masterVelocityRPS = voltage;
    }

    @Override
    public void setSpeed(double speed, double acceleration) {
        Logger.recordOutput("Shooter/simAutoSpeed", speed);
        voltage = speed;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}

