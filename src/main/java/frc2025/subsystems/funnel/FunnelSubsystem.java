package frc2025.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class FunnelSubsystem extends SubsystemBase {
    FunnelIO io;
    FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();
    boolean usingVoltage = false;

    public FunnelSubsystem(FunnelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Funnel", inputs);
        io.updateInputs(inputs);
    }

    // Sets speed as a decimal from 0-1
    public Command setSpeed(Supplier<Double> speed) {
        return this.run(() -> {
            usingVoltage = false;
            io.setSpeed(speed.get());
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.runOnce(() -> {
            usingVoltage = true;
            io.setVoltage(voltage.get());
        });
    }

    public Command setIntake(Supplier<Boolean> intake) {
        return this.run(() -> {
            usingVoltage = false;
            if (intake.get()) io.setSpeed(0.3);
            else io.setSpeed(0);
        });
    }

    public Command setOuttake (Supplier<Boolean> intake) {
        return this.run(() -> {
            usingVoltage = false;
            if (intake.get()) io.setSpeed(-0.3);
            else io.setSpeed(0);
        });
    }

    
}
