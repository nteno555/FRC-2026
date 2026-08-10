package frc2025.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterIOSim implements ShooterIO {
    private double voltage;
    private double percentOutput;
    private double current;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.alignedVelocityRPS = voltage;
        inputs.opposedAVelocityRPS = voltage;
        inputs.opposedBVelocityRPS = voltage;
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

    @Override
    public void setCurrent(double currentAmps) {
        this.current = currentAmps;
    }
}

