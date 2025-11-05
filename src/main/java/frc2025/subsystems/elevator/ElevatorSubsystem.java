package frc2025.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
    ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    double setpoint = 0.0;
    double voltageRamp = 0.0;
    boolean usingVoltage = false;

    public ElevatorSubsystem(ElevatorIO io) {
        io.rezero(0);
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Elevator", inputs);
        Logger.recordOutput("Elevator/setpoint", setpoint);

        if(DriverStation.isDisabled()) {
            setpoint = inputs.positionRotations;
        }

        io.updateInputs(inputs);
    }

    public Command setFixed() {
        return setPosition(() -> {
            if(usingVoltage) setpoint = inputs.positionRotations;
            return setpoint;
        });
    }

    public Command setPosition(Supplier<Double> meters) {
        return this.run(() -> {
            usingVoltage = false;
            setpoint = meters.get();
            io.setPosition(meters.get());
        });
    }

    public Command setPositionSlow(Supplier<Double> meters) {
        return this.run(() -> {
            usingVoltage = false;
            setpoint = meters.get();
            double voltage = MathUtil.clamp(0.2 * (setpoint - inputs.positionRotations), -3, 3);
            voltage = MathUtil.clamp(voltage, inputs.voltage-1, inputs.voltage+1);
            io.setVoltage(voltage);
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            io.setVoltage(voltage.get());
        });
    }

    public Command voltageRampCommand() {
        return this.runOnce(() -> { voltageRamp = 0.0; })
                        .andThen(
                                this.run(() -> {
                                    usingVoltage = true;
                                    io.setVoltage(voltageRamp);
                                    voltageRamp += 0.001;
                                }));
    }

    public Command rezero(double meters) {
        return this.runOnce(() -> io.rezero(meters));
    }

    public boolean isNear(double position) {
        return Math.abs(position - inputs.positionRotations) < 2.5;
    }

    public double getPosition() { return inputs.positionRotations; }
}
