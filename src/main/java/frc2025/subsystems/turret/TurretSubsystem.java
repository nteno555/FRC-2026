package frc2025.subsystems.turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
    TurretIO io;
    TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    double setpoint, voltage, voltageRamp;
    boolean usingVoltage = false;

    public TurretSubsystem(TurretIO io) {
        this.io = io;
        io.rezero();
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Turret", inputs);
        Logger.recordOutput("Turret/setpoint", setpoint);
        if(DriverStation.isDisabled()) {setpoint = inputs.positionTicks;}
        io.updateInputs(inputs);
    }

    public Command setPosition(Supplier<Double> position) {
        return this.run(() -> {
            setpoint = position.get();
            usingVoltage = false;
            io.setPosition(setpoint);
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            this.voltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command voltageRampCommand() {
        return this.runOnce(() -> { voltageRamp = 0.0; })
                .andThen(
                        this.run(() -> {
                            usingVoltage = true;
                            io.setVoltage(voltageRamp);
                            voltageRamp += 0.01;
                        }));
    }

    public Command rezero() {
        return this.run(() -> io.rezero());
    }

    public Command setFixed() {
        return setPosition(() -> {
            if(usingVoltage) setpoint = inputs.positionTicks;
            return setpoint;
        });
    }

    public double getPosition() { return inputs.positionTicks; }
    public double getRealAngle() { return inputs.positionDegreesReal; }
    public double getRealAngleRadians() { return inputs.positionRadiansReal; }
}