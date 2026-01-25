package frc2025.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {
    private double voltage;
    private double percentOutput;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.masterVoltage = voltage;
    }

    @Override
    public void setSpeed(double speed) {
        Logger.recordOutput("Shooter/simAutoSpeed", speed);
        voltage = speed;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void setPercentOutput(double percentOutput) {this.percentOutput = percentOutput;}

//    @Override
//    public boolean getShotReady(double wantedSpeed) {
//        return false;
//    }
}

